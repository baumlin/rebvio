/*
 * ros_rebvio.hpp
 *
 *  Created on: Nov 19, 2018
 *      Author: baumlin
 */

#ifndef ROS_REBVIO_INCLUDE_ROS_REBVIO_ROS_REBVIO_HPP_
#define ROS_REBVIO_INCLUDE_ROS_REBVIO_ROS_REBVIO_HPP_

#include "rebvio/rebvio.hpp"
#include "rebvio/types/odometry.hpp"

#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <tf/transform_broadcaster.h>

namespace rebvio {

/**
 * \class ROS Wrapper class for Rebvio. Provides the data interfaces for image and imu input, and odometry output.
 */
class RosRebvio {
public:
	RosRebvio(ros::NodeHandle _nh);
	RosRebvio() = delete;
	~RosRebvio();

	/*
	 * \brief Run the Rebvio pipeline
	 * \param _bag_file If a valid path to a bag file is specified, the pipeline uses the bag file for data input.
	 * \param _speed If _bag_file is specified, _speed determines the playback speed.
	 * \return True if bag file is finished processing or ROS spin has successfully shut down.
	 */
	bool run(std::string _bag_file = "", float _speed = 1.0);

	/*
	 * \brief Register a callback function to process the odometry output
	 */
	void registerOdometryCallback(std::function<void(rebvio::types::Odometry&)>);

private:
	/*
	 * \brief Image callback function binded to image subscriber
	 */
	void imageCallback(const sensor_msgs::ImageConstPtr& _msg);
	/*
	 * \brief IMU callback function binded to IMU subscriber
	 */
	void imuCallback(const sensor_msgs::ImuConstPtr& _msg);

private:
	rebvio::Rebvio rebvio_;                     //!< Rebvio instance

	ros::NodeHandle nh_;                        //!< ROS Node Handle
	image_transport::ImageTransport it_;        //!< Image Transport for image publishing
	image_transport::Publisher edge_image_pub_; //!< Edge Image Publisher
	tf::Transform tf_cam2robot_;                //!< Coordinate System transformation from robot to camera frame
	tf::TransformBroadcaster tf_broadcaster_;   //!< Transform broadcaster for odometry output
	image_transport::Subscriber image_sub_;     //!< Input image Subscriber
	ros::Subscriber imu_sub_;                   //!< Input IMU Subscriber

	std::string imu_topic_;                     //!< Name of IMU topic
	std::string cam_topic_;                     //!< Name of image topic
};

} /* namespace rebvio */



#endif /* ROS_REBVIO_INCLUDE_ROS_REBVIO_ROS_REBVIO_HPP_ */
