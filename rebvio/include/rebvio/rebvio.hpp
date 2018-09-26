/*
 * Rebvio.hpp
 *
 *  Created on: Aug 29, 2018
 *      Author: baumlin
 */

#ifndef REBVIO_HPP_
#define REBVIO_HPP_

#include "rebvio/rebvio_params.hpp"
#include "rebvio/edge_tracker.hpp"
#include "rebvio/camera.hpp"
#include "rebvio/types/image.hpp"
#include "rebvio/types/imu.hpp"

#include <mutex>
#include <thread>
#include <functional>
#include <queue>
#include <vector>
#include <opencv2/core.hpp>

namespace rebvio {

class Rebvio {
public:
	struct Odometry {
		uint64_t ts;
		rebvio::types::Matrix3f R;
		rebvio::types::Vector3f R_Lie;
		rebvio::types::Matrix3f Pose;
		rebvio::types::Vector3f Pose_Lie;
		rebvio::types::Vector3f position;
	};
public:
	Rebvio(rebvio::RebvioParams);
	~Rebvio();

	void imageCallback(rebvio::types::Image&&);
	void imuCallback(rebvio::types::Imu&&);

	void registerEdgeImageCallback(std::function<void(cv::Mat&,rebvio::types::EdgeMap::SharedPtr&)>);
	void registerOdometryCallback(std::function<void(rebvio::Rebvio::Odometry&)>);


private:
	void dataAcquisitionProcess();
	void edgeImageCallback(cv::Mat&,rebvio::types::EdgeMap::SharedPtr&);
	void odometryCallback(rebvio::Rebvio::Odometry&);

	void stateEstimationProcess();

private:
	rebvio::RebvioParams params_;
	std::thread data_acquisition_thread_;
	std::thread state_estimation_thread_;
	bool run_;

	uint num_frames_;

	rebvio::EdgeTracker edge_tracker_;

	rebvio::Camera camera_;
	rebvio::RebvioParams config_;
	rebvio::types::ImuState imu_state_;

	std::queue<rebvio::types::Image> image_buffer_;
	std::mutex image_buffer_mutex_;
	std::queue<rebvio::types::Imu> imu_buffer_;
	std::mutex imu_buffer_mutex_;
	std::queue<rebvio::types::EdgeMap::SharedPtr> edge_map_buffer_;
	std::mutex edge_map_buffer_mutex_;

	std::vector<std::function<void(cv::Mat&,rebvio::types::EdgeMap::SharedPtr&)>> edge_image_callbacks_;
	std::vector<std::function<void(rebvio::Rebvio::Odometry&)>> odometry_callbacks_;
};

} /* namespace rebvio */

#endif /* REBVIO_HPP_ */
