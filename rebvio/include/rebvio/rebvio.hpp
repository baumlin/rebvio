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
	Rebvio(rebvio::RebvioParams);
	~Rebvio();

	void imageCallback(rebvio::types::Image);
	void imuCallback(rebvio::types::Imu);

	void registerEdgeImageCallback(std::function<void(cv::Mat&,rebvio::types::EdgeMapPtr&)>);


private:
	void dataAcquisitionProcess();
	void edgeImageCallback(cv::Mat&,rebvio::types::EdgeMapPtr&);

	void stateEstimationProcess();

private:
	rebvio::RebvioParams params_;
	std::thread data_acquisition_thread_;
	std::thread state_estimation_thread_;
	bool run_;

	rebvio::EdgeTracker edge_tracker_;

	rebvio::Camera camera_;
	std::queue<rebvio::types::Image> image_buffer_;
	std::mutex image_buffer_mutex_;
	std::queue<rebvio::types::Imu> imu_buffer_;
	std::mutex imu_buffer_mutex_;
	std::queue<rebvio::types::EdgeMapPtr> edge_map_buffer_;
	std::mutex edge_map_buffer_mutex_;
	std::vector<std::function<void(cv::Mat&,rebvio::types::EdgeMapPtr&)>> edge_image_callbacks_;
};

} /* namespace rebvio */

#endif /* REBVIO_HPP_ */
