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
#include <tf/transform_broadcaster.h>

#include <memory>
#include <utility>
#include <TooN/TooN.h>


rebvio::RebvioParams params;
std::unique_ptr<rebvio::Rebvio> rebvio_ptr = std::unique_ptr<rebvio::Rebvio>(new rebvio::Rebvio(params));
image_transport::Publisher edge_image_pub;


void image_callback(const sensor_msgs::ImageConstPtr& _msg) {
	cv_bridge::CvImagePtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvCopy(_msg, sensor_msgs::image_encodings::MONO8);
		cv::Mat image, image_undistorted;
		cv_ptr->image.convertTo(image,CV_32FC1,3.0);
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
		TooN::makeVector((float)_msg->angular_velocity.x,(float)_msg->angular_velocity.y,(float)_msg->angular_velocity.z),
		TooN::makeVector((float)_msg->linear_acceleration.x,(float)_msg->linear_acceleration.y,(float)_msg->linear_acceleration.z)});
}

void edge_image_callback(cv::Mat& _edge_image,rebvio::types::EdgeMapPtr& _map) {
	cv::Mat image;
	if(_edge_image.type() != CV_8UC1) {
		double min=0,max=0;
		cv::minMaxLoc(_edge_image,&min,&max);
		_edge_image.convertTo(image,CV_8UC1,255.0/(max-min),-min*255.0/(max-min));
	} else {
		image = _edge_image;
	}
	cv::cvtColor(image,image,CV_GRAY2RGB);
	if(_map) {
		for(int i = 0; i < _map->size(); ++i) {
			image.at<cv::Vec3b>(std::round((*_map)[i].pos[1]),std::round((*_map)[i].pos[0])) = cv::Vec3b(255,0,0);
		}
	}
	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(),"rgb8",image).toImageMsg();
	edge_image_pub.publish(msg);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "ros_rebvio_node");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber image_sub = it.subscribe("/cam0/image_raw",20,&image_callback);
	ros::Subscriber imu_sub = nh.subscribe("/imu0",1,&imu_callback);

	edge_image_pub = it.advertise("edge_image",20);

	tf::TransformBroadcaster tf_broad;
	tf::Transform tf_cam2robot;
	tf_cam2robot.setOrigin(tf::Vector3(0,0,0));
	tf_cam2robot.setRotation(tf::Quaternion(tf::Vector3(1,0,0),M_PI/2));

	std::function<void(rebvio::Rebvio::Odometry&)> odometry_callback = [&](rebvio::Rebvio::Odometry& _odometry) {
		tf::Transform transform;
		tf::Vector3 rot(_odometry.Pose_Lie[0],_odometry.Pose_Lie[1],_odometry.Pose_Lie[2]);
		tfScalar angle = rot.length();
		if(angle > 0.0) transform.setRotation(tf::Quaternion(rot/angle,angle));
		else transform.setIdentity();
		transform.setOrigin(tf::Vector3(_odometry.position[0],_odometry.position[1],_odometry.position[2]));
		tf_broad.sendTransform(tf::StampedTransform(tf_cam2robot.inverse()*transform,ros::Time().fromNSec(_odometry.ts*1000),"map","rebvio_frame_cam"));
//		tf_broad.sendTransform(tf::StampedTransform(tf_cam2robot,ros::Time().fromNSec(_odometry.ts*1000),"rebvio_frame_cam","rebvio_frame_robot"));
	};
	rebvio_ptr->registerOdometryCallback(odometry_callback);


	rebvio_ptr->registerEdgeImageCallback(edge_image_callback);
	ros::spin();
}


