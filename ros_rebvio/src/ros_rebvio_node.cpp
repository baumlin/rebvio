/*
 * ros_rebvio_node.cpp
 *
 *  Created on: Aug 29, 2018
 *      Author: baumlin
 */

#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <memory>
#include <utility>
#include <TooN/TooN.h>

#include "ros_rebvio/ros_rebvio.hpp"


int main(int argc, char** argv) {
	ros::init(argc, argv, "ros_rebvio_node");
	ros::NodeHandle nh;
	ros::NodeHandle nhp("~");

	ROS_INFO_STREAM("Setting up "<<ros::this_node::getName()<<" Node");

	rebvio::RosRebvioConfig config;
	config.cam_topic = "/cam0/image_raw";
	config.imu_topic = "/imu0";
	rebvio::RosRebvio rebvio(nhp,config);

	std::string bag_file;
	nhp.getParam("bag_file",bag_file);
	rebvio.run(bag_file,1.5);
}


