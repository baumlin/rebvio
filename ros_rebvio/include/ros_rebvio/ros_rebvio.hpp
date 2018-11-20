/*
 * ros_rebvio.hpp
 *
 *  Created on: Nov 19, 2018
 *      Author: baumlin
 */

#ifndef ROS_REBVIO_INCLUDE_ROS_REBVIO_ROS_REBVIO_HPP_
#define ROS_REBVIO_INCLUDE_ROS_REBVIO_ROS_REBVIO_HPP_

#include "rebvio/rebvio.hpp"

#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <tf/transform_broadcaster.h>

namespace rebvio {

class RosRebvio {
public:
	RosRebvio(ros::NodeHandle _nh);
	~RosRebvio();

	bool run(std::string _bag_file = "");

	void registerOdometryCallback(std::function<void(rebvio::Rebvio::Odometry&)>);

private:
	void imageCallback(const sensor_msgs::ImageConstPtr& _msg);
	void imuCallback(const sensor_msgs::ImuConstPtr& _msg);

private:
	rebvio::Rebvio rebvio_;

	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Publisher edge_image_pub_;
	tf::Transform tf_cam2robot_;
	tf::TransformBroadcaster tf_broadcaster_;
	image_transport::Subscriber image_sub_;
	ros::Subscriber imu_sub_;

	std::string imu_topic_;
	std::string cam_topic_;
};

} /* namespace rebvio */



#endif /* ROS_REBVIO_INCLUDE_ROS_REBVIO_ROS_REBVIO_HPP_ */
