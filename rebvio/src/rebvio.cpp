/*
 * Rebvio.cpp
 *
 *  Created on: Aug 29, 2018
 *      Author: baumlin
 */

#include "rebvio/rebvio.hpp"
#include "rebvio/util/timer.hpp"
#include "rebvio/util/log.hpp"

#include <opencv2/imgproc.hpp>

#include <TooN/so3.h>
#include <TooN/Cholesky.h>


namespace rebvio {


Rebvio::Rebvio(rebvio::RebvioConfig& _config) :
		config_(_config),
		run_(true),
		num_frames_(0),
		edge_detector_(std::make_shared<rebvio::Camera>(camera_)),
		edge_tracker_(std::make_shared<rebvio::Camera>(camera_)),
		imu_state_(config_.imu_state_config_),
		sab_state_(config_.imu_state_config_),
		undistorter_(std::make_shared<rebvio::Camera>(camera_))
{
	util::Log::init();
	data_acquisition_thread_ = std::thread(&Rebvio::dataAcquisitionProcess,this);
	state_estimation_thread_ = std::thread(&Rebvio::stateEstimationProcess,this);
}

Rebvio::~Rebvio() {
	run_ = false;
	data_acquisition_thread_.join();
	state_estimation_thread_.join();
}

void Rebvio::imageCallback(rebvio::types::Image&& _image) {
	// add image to queue
	std::lock_guard<std::mutex> guard(image_buffer_mutex_);
	cv::Mat image_undistorted, img;

	// Use OpenCV undistorter class
	static types::Float K_data[9] = {camera_.fm_, 0.0, camera_.cx_, 0.0, camera_.fm_,camera_.cy_,0.0,0.0,1.0};
	static cv::Mat K = cv::Mat(3,3,CV_FLOAT_PRECISION,K_data);
	static types::Float D_data[5] = {camera_.k1_,camera_.k2_,camera_.p1_,camera_.p2_,camera_.k3_};
	static cv::Mat D = cv::Mat(1,5,CV_FLOAT_PRECISION,D_data);
	_image.data.convertTo(img,CV_FLOAT_PRECISION,3.0);
	REBVIO_TIMER_TICK();
	cv::undistort(img,image_undistorted,K,D);

  // Use own undistorter class
//  cv::cvtColor(_image.data,img,cv::COLOR_GRAY2RGB);
//	REBVIO_TIMER_TICK();
//	image_undistorted = undistorter_.undistort(img);

	REBVIO_TIMER_TOCK();
	_image.data = image_undistorted;
	image_buffer_.push(_image);
}

void Rebvio::imuCallback(rebvio::types::Imu&& _imu) {
	// add imu to queue
	std::lock_guard<std::mutex> guard(imu_buffer_mutex_);
	imu_buffer_.push(_imu);
}

void Rebvio::dataAcquisitionProcess() {
	// Detect Edges and integrate inter-frame IMU measurements
	REBVIO_INFO("Starting Data Acquisition Process..");
	while(run_) {
		if(!image_buffer_.empty()) {
			REBVIO_TIMER_TICK();

			rebvio::types::Image img;
			{
				std::lock_guard<std::mutex> guard(image_buffer_mutex_);
				// get front image
				img = image_buffer_.front();
				image_buffer_.pop();
			}
			rebvio::EdgeMap::SharedPtr edge_map = edge_detector_.detect(img);
			edgeImageCallback(img.data,edge_map);
			{
				std::lock_guard<std::mutex> guard(edge_map_buffer_mutex_);
				edge_map_buffer_.push(edge_map);
			}

			std::lock_guard<std::mutex> guard(imu_buffer_mutex_);
			while(!imu_buffer_.empty()) {
				if(imu_buffer_.front().ts <= img.ts_us) {
					// integrate imu measurements
					edge_map->imu().add(imu_buffer_.front(),camera_.getRc2i());
					imu_buffer_.pop();
				} else { break; }
			}
			REBVIO_TIMER_TOCK();
		} else {
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}
	}
}

void Rebvio::stateEstimationProcess() {
	REBVIO_INFO("Starting State Estimation Process..");

	types::Float K = 1.0; // Scale

	// estimated velocity, rotation (lie algebra) and position
	types::Vector3f V = TooN::Zeros;
	types::Vector3f W = TooN::Zeros;
	types::Vector3f Pos = TooN::Zeros;

	// Rotation matrix and global rotation
	types::Matrix3f R = TooN::Identity;     // Local rotation matrix
	types::Matrix3f R_global = TooN::Identity;  // Global rotation matrix

	// rotation matrix with filter correction (gva = gyro-visual-accelerometer)
	types::Matrix3f Rgva = TooN::Identity;

	types::Float P_Kp = 5e-6;  // Process noise of the angle scale
	types::Matrix3f P_V = TooN::Identity*std::numeric_limits<types::Float>::max();  // Process noise of the velocity
	types::Matrix3f P_W = TooN::Identity*1e-10;  // Process noise of the rotation

	int num_gyro_init = 0;
	types::Vector3f gyro_init = TooN::Zeros;
	types::Vector3f g_init = TooN::Zeros;



	while(run_) {
		REBVIO_TIMER_TICK();

		// access newest and old edge map
		rebvio::EdgeMap::SharedPtr new_edge_map, old_edge_map;
		{
			std::lock_guard<std::mutex> guard(edge_map_buffer_mutex_);
			if(edge_map_buffer_.size() >= 2) {
				old_edge_map = edge_map_buffer_.front();
				edge_map_buffer_.pop();
				new_edge_map = edge_map_buffer_.front();
			} else {
				REBVIO_TIMER_TOCK();
				continue;
			}
		}

		int klm_num = 0, num_kf_fow_m = 0, num_kf_back_m = 0;
		P_V = TooN::Identity*std::numeric_limits<types::Float>::max();
		P_W = TooN::Identity*std::numeric_limits<types::Float>::max();
		R = TooN::Identity;


		// build auxiliary distance field from new edge map for rigid transform estimation
		edge_tracker_.buildDistanceField(new_edge_map);

		// initialize imu state if frame received
		const rebvio::types::IntegratedImu& imu = new_edge_map->imu().get(camera_.getRc2i(),camera_.getTc2i());
		if(!imu_state_.initialized && num_frames_ > 0) {
			if(config_.imu_state_config_.init_bias > 0) {
				gyro_init += imu.gyro()*imu.dt_s();
				g_init -= imu.cacc();
				if(++num_gyro_init > config_.imu_state_config_.init_bias_frame_num) {
					imu_state_.Bg = gyro_init/num_gyro_init;                 // initialize the gyro bias
					imu_state_.W_Bg = types::invert(imu_state_.RGBias*1e2);  // initialize gyro bias information matrix
					sab_state_.X.slice<1,3>() = g_init/num_gyro_init;        // initialize bias state of the scale filter
					imu_state_.initialized = true;
				}
			} else {
				imu_state_.initialized = true;
				imu_state_.Bg = config_.imu_state_config_.init_bias_guess*imu.dt_s();
			}
		}

		// use imu inter-frame rotation to apply forward pre-rotation on old keylines for later matching with new keylines
		R = imu.R();
		R.T() = TooN::SO3<types::Float>(imu_state_.Bg)*R.T(); // correct inter-frame rotation with previously estimated gyro bias
		old_edge_map->rotateKeylines(R.T());									// (forward) rotate old keylines using rotation estimate from gyro measurements

		imu_state_.Vg = TooN::Zeros;
		// estimate translation and translation covariance using prior information from the gyro
		edge_tracker_.minimizeVel(old_edge_map,imu_state_.Vg,imu_state_.P_Vg);

		// forward keyline matching from the old (rotated) edge map to the new edge map
		old_edge_map->forwardMatch(new_edge_map);

		// Visual rigid transformation estimation using forward matches and prior translation estimate
		types::Vector6f Xv;                // Rigid body transformation correction from visual input
		types::Matrix6f W_Xv;
		edge_tracker_.extRotVel(new_edge_map,imu_state_.Vg,W_Xv,Xv);
		types::Vector6f Xgv = Xv;          // Estimated rigid transformation using gyro and visual input
		types::Matrix6f W_Xgv = W_Xv;


		// get inter-frame delta time in [s]
		types::Float frame_dt = types::Float(new_edge_map->ts_us()-old_edge_map->ts_us())/1000000.0; // convert to [s]

		// correct gyro bias
		imu_state_.RGBias = TooN::Identity*config_.imu_state_config_.gyro_bias_std_dev*config_.imu_state_config_.gyro_bias_std_dev*frame_dt*frame_dt;  // Observation noise of the gyro bias
		imu_state_.RGyro = TooN::Identity*config_.imu_state_config_.gyro_std_dev*config_.imu_state_config_.gyro_std_dev*frame_dt*frame_dt;             // Observation noise of the gyro measurements
		imu_state_.Bg += edge_tracker_.gyroBiasCorrection(Xgv,W_Xgv,imu_state_.W_Bg,imu_state_.RGyro,imu_state_.RGBias);
		imu_state_.dVgv = Xgv.slice<0,3>();  // Rigid body translation correction from visual input
		imu_state_.dWgv = Xgv.slice<3,3>();  // Rigid body rotation correction from visual input

		// extract rotation matrix
		Rgva = R;   // Estimated rigid body rotation using gyro, visual and accelerometer input
		TooN::SO3<types::Float> R0(imu_state_.dWgv);        // Rigid body (forward) rotation correction from visual input
		R.T() = R0.get_matrix()*R.T();                      // Correct gyro-estimated inter-frame rotation with visual information
		imu_state_.Vgv = R0*imu_state_.Vg+imu_state_.dVgv;  // Correct gyro-estimated translation with visual information
		V = imu_state_.Vgv;                                 // Update translation vector
		types::Matrix6f R_Xgv = TooN::Cholesky<6,types::Float>(W_Xgv).get_inverse();
		P_V = R_Xgv.slice<0,0,3,3>();
		P_W = R_Xgv.slice<3,3,3,3>();

		// Mix rigid body transformation estimate (from visual and gyro input) with accelerometer using bayesian filter
		edge_tracker_.estimateLs4Acceleration(-imu_state_.Vgv/frame_dt,imu_state_.Av,R,frame_dt);
		edge_tracker_.estimateMeanAcceleration(new_edge_map->imu().cacc(),imu_state_.As,R);

		types::Vector6f Xgva = Xgv; // Estimated rigid body transformation using gyro, visual, and accelerometer input
		sab_state_.Rv = P_V/(frame_dt*frame_dt*frame_dt*frame_dt); // Observation noise of the translation state
		sab_state_.Qrot = P_W; // Process noise of the rotation state
		sab_state_.QKp = P_Kp; // Process noise of the angle scale state
		if(num_frames_ > 4+config_.imu_state_config_.init_bias_frame_num) {
			K = edge_tracker_.estimateBias(imu_state_.As,imu_state_.Av,1.0,R,sab_state_.X,sab_state_.P,sab_state_.Qg,
																		 sab_state_.Qrot,sab_state_.Qbias,sab_state_.QKp,sab_state_.Rg,sab_state_.Rs,sab_state_.Rv,
																		 sab_state_.g_est,sab_state_.b_est,W_Xgv,Xgva,config_.imu_state_config_.g_norm);
			imu_state_.dVgva = Xgva.slice<0,3>();
			imu_state_.dWgva = Xgva.slice<3,3>();

			TooN::SO3<types::Float>R0gva(imu_state_.dWgva); // Rigid body (forward) rotation correction from accelerometer input
			Rgva.T() = R0gva.get_matrix()*Rgva.T();         // Correct gyro-visual estimated inter-frame rotation with accelerometer information
			imu_state_.Vgva = R0gva*imu_state_.Vg+imu_state_.dVgva; // Correct estimated translation with accelerometer information
			V = imu_state_.Vgva; // Update translation vector

			// forward rotate the old edge map points to new edge map with incremental rotation estimated with full visual, gyro and accelerometer input
			old_edge_map->rotateKeylines(R0gva.get_matrix());
		} else {
			imu_state_.dVgva = imu_state_.dVgv;
			imu_state_.dWgva = imu_state_.dWgv;
			Rgva = R;
			imu_state_.Vgva = imu_state_.Vgv;
			V = imu_state_.Vgva;

			// forward rotate the old edge map points to new edge map with incremental rotation estimated with full visual, gyro and accelerometer input
			old_edge_map->rotateKeylines(R0.get_matrix());
		}

		// check for minimization errors
		if(TooN::isnan(V) || TooN::isnan(W)) {
			P_V = TooN::Identity*std::numeric_limits<types::Float>::max();
			V = TooN::Zeros;
			P_Kp = std::numeric_limits<types::Float>::max();
			std::cerr<<"Minimization Error occured!\n";
			run_ = false;
		} else {

			// match from the new edge map to the old one searching on the stereo line
			klm_num = new_edge_map->directedMatch(old_edge_map,V,P_V,Rgva,num_kf_back_m,edge_tracker_.config()->search_range);

			if(klm_num < edge_tracker_.config()->global_min_matches_threshold) {
				P_V = TooN::Identity*std::numeric_limits<types::Float>::max();
				V = TooN::Zeros;
				P_Kp = std::numeric_limits<types::Float>::max();
				std::cerr<<"Insufficient number of keylines matches!\n";
				run_ = false;
			} else {

				// regularize edge map depth
				new_edge_map->regularize1Iter();

				// improve depth using kalman filter
				edge_tracker_.updateInverseDepth(V);
			}
		}
		// estimate position and pose incrementally
		if(num_frames_ > 4+config_.imu_state_config_.init_bias_frame_num) {
			imu_state_.u_est = Rgva.T()*imu_state_.u_est;
			imu_state_.u_est = imu_state_.u_est-(imu_state_.u_est*sab_state_.g_est)/(sab_state_.g_est*sab_state_.g_est)*sab_state_.g_est;
			TooN::normalize(imu_state_.u_est);
			types::Matrix3f R1 = TooN::SO3<types::Float>(sab_state_.g_est,TooN::makeVector(0.0f,1.0f,0.0f)).get_matrix();
			types::Matrix3f R2 = TooN::SO3<types::Float>(R1*imu_state_.u_est,TooN::makeVector(1.0f,0.0f,0.0f)).get_matrix();
			R_global = R2*R1;
			Pos += -R_global*imu_state_.Vgva*K;
		}

		Odometry odometry;
		odometry.ts_us = new_edge_map->ts_us();
		odometry.orientation = TooN::SO3<types::Float>(R_global).ln();
		odometry.position = Pos;
		odometryCallback(odometry);
		REBVIO_ODOMETRY("{} {} {} {} {} {} {}",odometry.ts_us,
				                                   odometry.orientation[0],odometry.orientation[1],odometry.orientation[2],
																					 odometry.position[0],odometry.position[1],odometry.position[2]);


		++num_frames_;
		REBVIO_TIMER_TOCK();
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}
}

void Rebvio::edgeImageCallback(cv::Mat& _edge_image, rebvio::EdgeMap::SharedPtr& _map) {
	for(auto& cb : edge_image_callbacks_)
		cb(_edge_image,_map);
}

void Rebvio::registerEdgeImageCallback(std::function<void(cv::Mat&,rebvio::EdgeMap::SharedPtr&)> _cb) {
	edge_image_callbacks_.push_back(_cb);
}

void Rebvio::odometryCallback(rebvio::Rebvio::Odometry& _odometry) {
	for(auto& cb : odometry_callbacks_)
		cb(_odometry);
}

void Rebvio::registerOdometryCallback(std::function<void(rebvio::Rebvio::Odometry&)> _cb) {
	odometry_callbacks_.push_back(_cb);
}

} /* namespace rebvio */
