/*
 * ros_rebvio_node.cpp
 *
 *  Created on: Aug 29, 2018
 *      Author: baumlin
 */

#include <ros/ros.h>
#include <rebvio/rebvio_params.hpp>
#include <rebvio/rebvio.hpp>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <memory>
#include <utility>


rebvio::RebvioParams params;
std::unique_ptr<rebvio::Rebvio> rebvio_ptr = std::unique_ptr<rebvio::Rebvio>(new rebvio::Rebvio(params));

void image_callback(const sensor_msgs::ImageConstPtr& _msg) {
	cv_bridge::CvImagePtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvCopy(_msg, sensor_msgs::image_encodings::MONO8);
		cv::Mat image;
		cv_ptr->image.convertTo(image,CV_32FC1);
		rebvio_ptr->imageCallback(rebvio::types::Image{(_msg->header.stamp.toNSec())/1000,image});
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
}

void imu_callback(const sensor_msgs::ImuConstPtr& _msg) {
	rebvio_ptr->imuCallback(rebvio::types::Imu{_msg->header.stamp.toNSec()/1000,
		Eigen::Vector3f((float)_msg->angular_velocity.x,(float)_msg->angular_velocity.y,(float)_msg->angular_velocity.z),
		Eigen::Vector3f((float)_msg->linear_acceleration.x,(float)_msg->linear_acceleration.y,(float)_msg->linear_acceleration.z)});
}




int main(int argc, char** argv) {
	ros::init(argc, argv, "ros_rebvio_node");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber image_sub = it.subscribe("/cam0/image_raw",20,&image_callback);
	ros::Subscriber imu_sub = nh.subscribe("/imu0",1,&imu_callback);

	image_transport::Publisher edge_image_pub = it.advertise("edge_image",20);
	std::function<void(cv::Mat&,rebvio::types::EdgeMapPtr&)> edge_image_callback = [&](cv::Mat& _edge_image,rebvio::types::EdgeMapPtr& _map) {
		double min=0,max=0;
		cv::minMaxLoc(_edge_image,&min,&max);
		cv::Mat image;
		_edge_image.convertTo(image,CV_8UC1,255.0/(max-min),-min*255.0/(max-min));
		cv::cvtColor(image,image,CV_GRAY2RGB);
		for(int i = 0; i < _map->size(); ++i) {
			image.at<cv::Vec3b>(std::round((*_map)[i].pos(1)),std::round((*_map)[i].pos(0))) = cv::Vec3b(255,0,0);
		}
		sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(),"rgb8",image).toImageMsg();
		edge_image_pub.publish(msg);
	};
	rebvio_ptr->registerEdgeImageCallback(edge_image_callback);
	ros::spin();
}


