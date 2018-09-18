/*
 * Rebvio.cpp
 *
 *  Created on: Aug 29, 2018
 *      Author: baumlin
 */

#include "rebvio/rebvio.hpp"
#include "rebvio/scale_space.hpp"

#include <iostream>
#include <chrono>
#include <opencv2/imgproc.hpp>


namespace rebvio {

Rebvio::Rebvio() {
	// TODO Auto-generated constructor stub

}

Rebvio::Rebvio(rebvio::RebvioParams _params) : params_(_params), run_(true) {
	data_acquisition_thread_ = std::thread(&Rebvio::dataAcquisitionProcess,this);
	state_estimation_thread_ = std::thread(&Rebvio::stateEstimationProcess,this);
}

Rebvio::~Rebvio() {
	run_ = false;
	data_acquisition_thread_.join();
	state_estimation_thread_.join();
}

void Rebvio::imageCallback(rebvio::types::Image _image) {
	// add image to queue
	std::lock_guard<std::mutex> guard(image_buffer_mutex_);
	image_buffer_.push(_image);
}

void Rebvio::imuCallback(rebvio::types::Imu _imu) {
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
				// get front image (pair)
				img = image_buffer_.front();
				image_buffer_.pop();
			}
			rebvio::types::EdgeMapPtr edge_map = edge_tracker_.detect(img,100);
			edgeImageCallback(img.data,edge_map);
			{
				std::lock_guard<std::mutex> guard(edge_map_buffer_mutex_);
				edge_map_buffer_.push(edge_map);
			}

			std::lock_guard<std::mutex> guard(imu_buffer_mutex_);
			rebvio::types::IntegratedImu imu;
			while(!imu_buffer_.empty()) {
				float dt = 0.005;
				if(imu_buffer_.front().ts <= img.ts) {
					// integrate imu measurements
					imu.add(imu_buffer_.front(),camera_.getRc2i(),dt);
					imu_buffer_.pop();
				}
				else break;
			}
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(1));

	}
}

void Rebvio::stateEstimationProcess() {
	int num_frame = 0;
	float Kp = 1, K = 1;
	float error_vel = 0.0;
	float error_score = 0.0;

	while(run_) {
		int klm_num = 0, num_kf_fow_m = 0, num_kf_back_m = 0;
		// access newest and old edge map
		rebvio::types::EdgeMapPtr new_edge_map, old_edge_map;
		{
			std::lock_guard<std::mutex> guard(edge_map_buffer_mutex_);
			if(edge_map_buffer_.size() >= 2) {
				old_edge_map = edge_map_buffer_.front();
				edge_map_buffer_.pop();
				new_edge_map = edge_map_buffer_.front();
			} else continue;
		}

		uint64_t frame_dt = new_edge_map->ts()-old_edge_map->ts();

		float sigma_rho_q = old_edge_map->estimateQuantile(rebvio::types::RHO_MIN,rebvio::types::RHO_MAX,0.9,100); // TODO: qcutoffquantile=0.9, qcutoffnumbins=100

		// build auxiliary distance field from edge map

		// use imu rotation to apply forward pre-rotation on old keylines
		// estimate translation
		// match from the old edge map to the new using the information from the previous minimization
		// visual roto-translation estimation using forward matches
		// correct biases
		// extract rotation matrix
		// mix with accelerometer using bayesian filter

		// forward rotate the old edge map points

		// push into pose graph

		// check for minimization errors

		// match from the new edge map to the old one searching on the stereo line

		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
}

void Rebvio::edgeImageCallback(cv::Mat& _edge_image, rebvio::types::EdgeMapPtr& _map) {
	for(auto& cb : edge_image_callbacks_)
		cb(_edge_image,_map);
}

void Rebvio::registerEdgeImageCallback(std::function<void(cv::Mat&,rebvio::types::EdgeMapPtr&)> _cb) {
	edge_image_callbacks_.push_back(_cb);
}

} /* namespace rebvio */
