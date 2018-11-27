/*
 * Rebvio.hpp
 *
 *  Created on: Aug 29, 2018
 *      Author: baumlin
 */

#ifndef REBVIO_HPP_
#define REBVIO_HPP_

#include "rebvio/edge_detector.hpp"
#include "rebvio/edge_tracker.hpp"
#include "rebvio/camera.hpp"
#include "rebvio/types/definitions.hpp"
#include "rebvio/types/image.hpp"
#include "rebvio/types/imu.hpp"
#include "rebvio/types/odometry.hpp"
#include "rebvio/sab_estimator.hpp"

#include <mutex>
#include <thread>
#include <functional>
#include <queue>
#include <vector>
#include <opencv2/core.hpp>

namespace rebvio {

struct RebvioConfig {

	// IMU
	rebvio::types::ImuStateConfig imu_state_config_;

};

class Rebvio {

public:
	Rebvio(rebvio::RebvioConfig&& _params);
	~Rebvio();

	void imageCallback(rebvio::types::Image&&);
	void imuCallback(rebvio::types::Imu&&);

	void registerEdgeImageCallback(std::function<void(cv::Mat&,rebvio::EdgeMap::SharedPtr&)>);
	void registerOdometryCallback(std::function<void(rebvio::types::Odometry&)>);


private:
	void dataAcquisitionProcess();
	void edgeImageCallback(cv::Mat&,rebvio::EdgeMap::SharedPtr&);
	void odometryCallback(rebvio::types::Odometry&);

	void stateEstimationProcess();

private:
	rebvio::RebvioConfig config_;
	std::thread data_acquisition_thread_;
	std::thread state_estimation_thread_;
	bool run_;

	unsigned int num_frames_;

	rebvio::Camera camera_;
	rebvio::EdgeDetector edge_detector_;
	rebvio::EdgeTracker edge_tracker_;
	rebvio::types::ImuState imu_state_;
	rebvio::SABEstimator::State sab_state_;

	std::queue<rebvio::types::Image> image_buffer_;
	std::mutex image_buffer_mutex_;
	std::queue<rebvio::types::Imu> imu_buffer_;
	std::mutex imu_buffer_mutex_;
	std::queue<rebvio::EdgeMap::SharedPtr> edge_map_buffer_;
	std::mutex edge_map_buffer_mutex_;

	std::vector<std::function<void(cv::Mat&,rebvio::EdgeMap::SharedPtr&)>> edge_image_callbacks_;
	std::vector<std::function<void(rebvio::types::Odometry&)>> odometry_callbacks_;
};

} /* namespace rebvio */

#endif /* REBVIO_HPP_ */
