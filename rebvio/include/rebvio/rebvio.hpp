/*
 * Rebvio.hpp
 *
 *  Created on: Aug 29, 2018
 *      Author: baumlin
 */

#ifndef REBVIO_HPP_
#define REBVIO_HPP_

#include "rebvio/edge_detector.hpp"
#include "rebvio/camera.hpp"
#include "rebvio/types/definitions.hpp"
#include "rebvio/types/image.hpp"
#include "rebvio/types/imu.hpp"
#include "rebvio/types/odometry.hpp"
#include "rebvio/sab_estimator.hpp"
#include "rebvio/core.hpp"

#include <mutex>
#include <thread>
#include <functional>
#include <queue>
#include <vector>
#include <opencv2/core.hpp>

namespace rebvio {

struct RebvioConfig {
	rebvio::EdgeDetectorConfig edge_detector;  //!< Edge Detector Configuration
	rebvio::CoreConfig core;                   //!< Rebvio Core Configuration
	rebvio::types::ImuStateConfig imu_state;   //!< IMU State Configuration
};

/**
 * \class Rebvio: Real-time Edge-based Visual-Inertial Odometry Pipeline
 */
class Rebvio {

public:
	Rebvio(rebvio::RebvioConfig& _config);
	~Rebvio();

	/**
	 * \brief Image Callback Function: Undistorts the input image and adds it to the internal image queue.
	 */
	void imageCallback(rebvio::types::Image&& _img);

	/**
	 * \brief IMU Callback Function: Adds the IMU measurement to the internal IMU queue.
	 */
	void imuCallback(rebvio::types::Imu&& _imu);

	/**
	 * \brief Register a callback function to process every generated Edge Image
	 */
	void registerEdgeImageCallback(std::function<void(cv::Mat&,rebvio::EdgeMap::SharedPtr&)>);

	/**
	 * \brief Register a callback function to process the odometry output
	 */
	void registerOdometryCallback(std::function<void(rebvio::types::Odometry&)>);


private:
	/**
	 * \brief Data Acquisition Method: Retrieves and processes Input Data from the Input Queues
	 */
	void dataAcquisitionProcess();

	/**
	 * \brief State Estimation Method: Estimates the State from Edge Maps
	 */
	void stateEstimationProcess();

	/**
	 * \brief Call all registered Edge Image Callback Functions
	 * \param _img Image to draw Edge Map
	 * \param _map Edge Map to draw into _img
	 */
	void edgeImageCallback(cv::Mat& _img,rebvio::EdgeMap::SharedPtr& _map);

	/**
	 * \brief Call all registered Odometry Callback Functions
	 * \param _msg Odometry measurement Input for the Callback Functions
	 */
	void odometryCallback(rebvio::types::Odometry& _msg);


private:
	rebvio::RebvioConfig config_;                               //!< Configuration Struct
	std::thread data_acquisition_thread_;                       //!< Thread that runs the Data Acquisition Process
	std::thread state_estimation_thread_;                       //!< Thread that runs the State Estimation Processs
	bool run_;                                                  //!< Status Flag

	unsigned int num_frames_;                                   //!< Number of processed Images/Edge Maps

	rebvio::Camera camera_;                                     //!< Camera parameters
	rebvio::EdgeDetector edge_detector_;                        //!< Edge Detector
	rebvio::Core core_;                                         //!< Pose Estimator
	rebvio::types::ImuState imu_state_;                         //!< Integrated IMU State
	rebvio::SABEstimator::State sab_state_;                     //!< Scale-Attitude-Bias Estimator State

	std::queue<rebvio::types::Image> image_buffer_;             //!< Input Image Queue
	std::mutex image_buffer_mutex_;                             //!< Mutex for locking the Image Queue
	std::queue<rebvio::types::Imu> imu_buffer_;                 //!< Input IMU Queue
	std::mutex imu_buffer_mutex_;                               //!< Mutex for locking the IMU Queue
	std::queue<rebvio::EdgeMap::SharedPtr> edge_map_buffer_;    //!< Edge Map (processed from input images) Queue for Pose Estimator
	std::mutex edge_map_buffer_mutex_;                          //!< Mutex for locking the Edge Map Queue

	std::vector<std::function<void(cv::Mat&,rebvio::EdgeMap::SharedPtr&)>> edge_image_callbacks_; //!< Vector of callback functions called for an Edge Image
	std::vector<std::function<void(rebvio::types::Odometry&)>> odometry_callbacks_;               //!< Vector of callback functions called for an Odometry Output
};

} /* namespace rebvio */

#endif /* REBVIO_HPP_ */
