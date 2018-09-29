/*
 * Rebvio.cpp
 *
 *  Created on: Aug 29, 2018
 *      Author: baumlin
 */

#include "rebvio/rebvio.hpp"
#include "rebvio/scale_space.hpp"
#include "rebvio/util/timer.hpp"

#include <iostream>
#include <chrono>
#include <opencv2/imgproc.hpp>

#include <TooN/so3.h>
#include <TooN/Cholesky.h>


namespace rebvio {


Rebvio::Rebvio(rebvio::RebvioParams _params) :
		params_(_params),
		run_(true),
		num_frames_(0),
		edge_tracker_(std::make_shared<rebvio::Camera>(camera_)),
		imu_state_(_params.imu_state_config_),
		undistorter_(std::make_shared<rebvio::Camera>(camera_))
{
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
//	static float K_data[9] = {camera_.fm_, 0.0, camera_.cx_, 0.0, camera_.fm_,camera_.cy_,0.0,0.0,1.0};
//	static cv::Mat K = cv::Mat(3,3,CV_32FC1,K_data);
//	static float D_data[5] = {camera_.k1_,camera_.k2_,camera_.p1_,camera_.p2_,camera_.k3_};
//	static cv::Mat D = cv::Mat(1,5,CV_32FC1,D_data);
	cv::Mat image_undistorted;
//	cv::undistort(_image.data,image_undistorted,K,D);
	image_undistorted = undistorter_.undistort(_image.data);
	_image.data = image_undistorted;
	image_buffer_.push(_image);
}

void Rebvio::imuCallback(rebvio::types::Imu&& _imu) {
	// add imu to queue
	std::lock_guard<std::mutex> guard(imu_buffer_mutex_);
	imu_buffer_.push(_imu);
}

void Rebvio::dataAcquisitionProcess() {
	// Build the scale spaces and the DoG for edge detection
	// Detect Edges
	// Integrate Imu
	while(run_) {
		if(!image_buffer_.empty()) {
			rebvio::types::Image img;
			{
				std::lock_guard<std::mutex> guard(image_buffer_mutex_);
				// get front image
				img = image_buffer_.front();
				image_buffer_.pop();
			}
			rebvio::types::EdgeMap::SharedPtr edge_map = edge_tracker_.detect(img,100);
			edgeImageCallback(img.data,edge_map);
			{
				std::lock_guard<std::mutex> guard(edge_map_buffer_mutex_);
				edge_map_buffer_.push(edge_map);
			}

			std::lock_guard<std::mutex> guard(imu_buffer_mutex_);
			while(!imu_buffer_.empty()) {
				if(imu_buffer_.front().ts <= img.ts) {
					// integrate imu measurements
					edge_map->imu().add(imu_buffer_.front(),camera_.getRc2i());
					imu_buffer_.pop();
				} else { break; }
			}
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(1));

	}
}

void Rebvio::stateEstimationProcess() {
	float Kp = 1.0, K = 1.0;
	float P_Kp = 5e-6;

	// estimated velocity, rotation and position
	types::Vector3f V = TooN::Zeros;
	types::Vector3f W = TooN::Zeros;
	types::Vector3f Pos = TooN::Zeros;

	// Rotation matrix and global rotation
	types::Matrix3f R = TooN::Identity;
	types::Matrix3f Pose = TooN::Identity;

	// rotation matrix with filter correction
	types::Matrix3f Rgva = TooN::Identity;

	types::Matrix3f P_V = TooN::Identity*std::numeric_limits<float>::max();
	types::Matrix3f P_W = TooN::Identity*1e-10;

	int num_gyro_init = 0;
	types::Vector3f gyro_init = TooN::Zeros;
	types::Vector3f g_init = TooN::Zeros;



	while(run_) {

		// access newest and old edge map
		rebvio::types::EdgeMap::SharedPtr new_edge_map, old_edge_map;
		{
			std::lock_guard<std::mutex> guard(edge_map_buffer_mutex_);
			if(edge_map_buffer_.size() >= 2) {
				old_edge_map = edge_map_buffer_.front();
				edge_map_buffer_.pop();
				new_edge_map = edge_map_buffer_.front();
			} else continue;
		}

		int klm_num = 0, num_kf_fow_m = 0, num_kf_back_m = 0;
		P_V = TooN::Identity*std::numeric_limits<float>::max();
		P_W = TooN::Identity*std::numeric_limits<float>::max();
		R = TooN::Identity;


		// build auxiliary distance field from edge map
		edge_tracker_.buildDistanceField(new_edge_map);

		// initialize imu state if frame received
		const rebvio::types::IntegratedImu& imu = new_edge_map->imu().get(camera_.getRc2i(),camera_.getTc2i());
		if(!imu_state_.initialized && num_frames_ > 0) {
			if(config_.imu_state_config_.init_bias > 0) {
				gyro_init += imu.gyro()*imu.dt_s();
				g_init -= imu.cacc();
				if(++num_gyro_init > config_.imu_state_config_.init_bias_frame_num) {
					imu_state_.Bg = gyro_init/num_gyro_init;
					imu_state_.W_Bg = types::invert(imu_state_.RGBias*1e2);
					imu_state_.X.slice<1,3>() = g_init/num_gyro_init;
					imu_state_.initialized = true;
				}
			} else {
				imu_state_.initialized = true;
				imu_state_.Bg = config_.imu_state_config_.init_bias_guess*imu.dt_s();
			}
		}

		// use imu rotation to apply forward pre-rotation on old keylines
		R = imu.R();
		R.T() = TooN::SO3<float>(imu_state_.Bg)*R.T(); // TODO: what is this?
		old_edge_map->rotateKeylines(R.T());

		imu_state_.Vg = TooN::Zeros;
		// estimate translation
		edge_tracker_.minimizeVel(old_edge_map,imu_state_.Vg,imu_state_.P_Vg);

		// match from the old edge map to the new using the information from the previous minimization
		old_edge_map->forwardMatch(new_edge_map);

		// visual roto-translation estimation using forward matches
		types::Matrix6f R_Xv, R_Xgv, W_Xv, W_Xgv;
		types::Vector6f Xv, Xgv, Xgva;
		edge_tracker_.extRotVel(new_edge_map,imu_state_.Vg,W_Xv,R_Xv,Xv);

		imu_state_.dVv = Xv.slice<0,3>();
		imu_state_.dWv = Xv.slice<3,3>();
		Xgv = Xv;
		W_Xgv = W_Xv;


		float frame_dt = float(new_edge_map->ts()-old_edge_map->ts())/1000000.0; // convert to [s]

		// correct biases
		imu_state_.RGBias = TooN::Identity*config_.imu_state_config_.gyro_bias_std_dev*config_.imu_state_config_.gyro_bias_std_dev*frame_dt*frame_dt;
		imu_state_.RGyro = TooN::Identity*config_.imu_state_config_.gyro_std_dev*config_.imu_state_config_.gyro_std_dev*frame_dt*frame_dt;
		types::Vector3f dgbias = TooN::Zeros;
		edge_tracker_.correctBias(Xgv,W_Xgv,dgbias,imu_state_.W_Bg,imu_state_.RGyro,imu_state_.RGBias);
		imu_state_.Bg += dgbias;
		imu_state_.dVgv = Xgv.slice<0,3>();
		imu_state_.dWgv = Xgv.slice<3,3>();

		// extract rotation matrix
		Rgva = R;
		TooN::SO3<float> R0(imu_state_.dWgv);
		R.T() = R0.get_matrix()*R.T();
		imu_state_.Vgv = R0*imu_state_.Vg+imu_state_.dVgv;
		V = imu_state_.Vgv;
		imu_state_.Wgv = TooN::SO3<float>(R).ln();
		R_Xgv = TooN::Cholesky<6,float>(W_Xgv).get_inverse();
		P_V = R_Xgv.slice<0,0,3,3>();
		P_W = R_Xgv.slice<3,3,3,3>();

		// mix with accelerometer using bayesian filter
		edge_tracker_.estimateLs4Acceleration(-imu_state_.Vgv/frame_dt,imu_state_.Av,R,frame_dt);
		edge_tracker_.estimateMeanAcceleration(new_edge_map->imu().cacc(),imu_state_.As,R);

		Xgva = Xgv;
		imu_state_.Rv = P_V/(frame_dt*frame_dt*frame_dt*frame_dt);
		imu_state_.Qrot = P_W;
		imu_state_.QKp = P_Kp;
		if(num_frames_ > 4+config_.imu_state_config_.init_bias_frame_num) {
			K = edge_tracker_.estimateBias(imu_state_.As,imu_state_.Av,1.0,R,imu_state_.X,imu_state_.P,imu_state_.Qg,
																		 imu_state_.Qrot,imu_state_.Qbias,imu_state_.QKp,imu_state_.Rg,imu_state_.Rs,imu_state_.Rv,
																		 imu_state_.g_est,imu_state_.b_est,W_Xgv,Xgva,config_.imu_state_config_.g_module);
			imu_state_.dVgva = Xgva.slice<0,3>();
			imu_state_.dWgva = Xgva.slice<3,3>();

			TooN::SO3<float>R0gva(imu_state_.dWgva);
			Rgva.T() = R0gva.get_matrix()*Rgva.T();
			imu_state_.Vgva = R0gva*imu_state_.Vg+imu_state_.dVgva;
		} else {
			imu_state_.dVgva = imu_state_.dVgv;
			imu_state_.dWgva = imu_state_.dWgv;
			Rgva = R;
			imu_state_.Vgva = imu_state_.Vgv;
		}

		// forward rotate the old edge map points
		old_edge_map->rotateKeylines(R0.get_matrix());

		// check for minimization errors
		if(TooN::isnan(V) || TooN::isnan(W)) {
			P_V = TooN::Identity*std::numeric_limits<float>::max();
			V = TooN::Zeros;
			Kp = 1.0;
			P_Kp = std::numeric_limits<float>::max();
			std::cerr<<"Minimization Error occured!\n";
			run_ = false;
		} else {

			// match from the new edge map to the old one searching on the stereo line
			klm_num = new_edge_map->directedMatch(old_edge_map,V,P_V,R,num_kf_back_m,
					edge_tracker_.config().match_threshold_module,edge_tracker_.config().match_threshold_angle,edge_tracker_.config().search_range,
					edge_tracker_.config().pixel_uncertainty_match);

			if(klm_num < edge_tracker_.config().global_min_matches_threshold) {
				P_V = TooN::Identity*std::numeric_limits<float>::max();
				V = TooN::Zeros;
				Kp = 1.0;
				P_Kp = std::numeric_limits<float>::max();
				std::cerr<<"Insufficient number of keylines matches!\n";
				run_ = false;
			} else {

				// regularize edgemap depth
				new_edge_map->regularize1Iter(edge_tracker_.config().regularization_threshold);

				// improve depth using kalman filter
				edge_tracker_.updateInverseDepth(V);
			}
		}
		// estimate position and pose incrementally
		if(num_frames_ > 4+config_.imu_state_config_.init_bias_frame_num) {
			imu_state_.u_est = Rgva.T()*imu_state_.u_est;
			imu_state_.u_est = imu_state_.u_est-(imu_state_.u_est*imu_state_.g_est)/(imu_state_.g_est*imu_state_.g_est)*imu_state_.g_est;
			TooN::normalize(imu_state_.u_est);
			types::Matrix3f PoseP1 = TooN::SO3<float>(imu_state_.g_est,TooN::makeVector(0.0f,1.0f,0.0f)).get_matrix();
			types::Matrix3f PoseP2 = TooN::SO3<float>(PoseP1*imu_state_.u_est,TooN::makeVector(1.0f,0.0f,0.0f)).get_matrix();
			Pose = PoseP2*PoseP1;
			Pos += -Pose*imu_state_.Vgva*K;
			imu_state_.Posgva = Pos;
			imu_state_.Posgv += -Pose*imu_state_.Vgv*K;
		}
		P_V /= frame_dt*frame_dt;

		Odometry odometry;
		odometry.ts = new_edge_map->ts();
		odometry.R = R;
		odometry.R_Lie = TooN::SO3<float>(R).ln();
		odometry.Pose = Pose;
		odometry.Pose_Lie = TooN::SO3<float>(Pose).ln();
		odometry.position = Pos;
		odometryCallback(odometry);


		++num_frames_;
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}
}

void Rebvio::edgeImageCallback(cv::Mat& _edge_image, rebvio::types::EdgeMap::SharedPtr& _map) {
	for(auto& cb : edge_image_callbacks_)
		cb(_edge_image,_map);
}

void Rebvio::registerEdgeImageCallback(std::function<void(cv::Mat&,rebvio::types::EdgeMap::SharedPtr&)> _cb) {
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
