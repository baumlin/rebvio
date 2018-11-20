/*
 * ros_rebvio.cpp
 *
 *  Created on: Nov 19, 2018
 *      Author: baumlin
 */

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include "ros_rebvio/ros_rebvio.hpp"

namespace rebvio {

RosRebvio::RosRebvio(ros::NodeHandle _nh) : nh_(_nh), it_(_nh), rebvio_(rebvio::RebvioConfig()) {

	tf_cam2robot_.setOrigin(tf::Vector3(0,0,0));
	tf_cam2robot_.setRotation(tf::Quaternion(tf::Vector3(1,0,0),M_PI/2));


	std::function<void(rebvio::Rebvio::Odometry& _odometry)> odometryCallback = [&](rebvio::Rebvio::Odometry& _odometry) {
		tf::Transform transform;
		tf::Vector3 rot(_odometry.orientation[0],_odometry.orientation[1],_odometry.orientation[2]);
		tfScalar angle = rot.length();
		if(angle > 0.0) transform.setRotation(tf::Quaternion(rot/angle,angle));
		else transform.setIdentity();
		transform.setOrigin(tf::Vector3(_odometry.position[0],_odometry.position[1],_odometry.position[2]));
		tf_broadcaster_.sendTransform(tf::StampedTransform(tf_cam2robot_.inverse()*transform,ros::Time().fromNSec(_odometry.ts_us*1000),"map","rebvio_frame_cam"));
	//		tf_broad.sendTransform(tf::StampedTransform(tf_cam2robot,ros::Time().fromNSec(_odometry.ts*1000),"rebvio_frame_cam","rebvio_frame_robot"));
	};
	rebvio_.registerOdometryCallback(odometryCallback);

	std::function<void(cv::Mat& _edge_image,rebvio::EdgeMap::SharedPtr& _map)> edgeImageCallback = [&](cv::Mat& _edge_image,rebvio::EdgeMap::SharedPtr& _map) {
//	void RosRebvio::edgeImageCallback(cv::Mat& _edge_image,rebvio::EdgeMap::SharedPtr& _map) {
		if(edge_image_pub_.getNumSubscribers() == 0) return;
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
		edge_image_pub_.publish(msg);
	};
	rebvio_.registerEdgeImageCallback(edgeImageCallback);

	nh_.param<std::string>("cam_topic",cam_topic_,"/cam0/image_raw");
	nh_.param<std::string>("imu_topic",imu_topic_,"/imu0");

	image_sub_ = it_.subscribe(cam_topic_,20,&rebvio::RosRebvio::imageCallback,this);
	imu_sub_ = nh_.subscribe(imu_topic_,200,&rebvio::RosRebvio::imuCallback,this);
	edge_image_pub_ = it_.advertise("edge_image",20);

}

RosRebvio::~RosRebvio() {
  edge_image_pub_.shutdown();
}

void RosRebvio::imageCallback(const sensor_msgs::ImageConstPtr& _msg) {
	cv_bridge::CvImagePtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvCopy(_msg, sensor_msgs::image_encodings::MONO8);
		rebvio_.imageCallback(rebvio::types::Image{(_msg->header.stamp.toNSec())/1000,cv_ptr->image});
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
}

void RosRebvio::imuCallback(const sensor_msgs::ImuConstPtr& _msg) {
	rebvio_.imuCallback(rebvio::types::Imu{_msg->header.stamp.toNSec()/1000,
		TooN::makeVector((rebvio::types::Float)_msg->angular_velocity.x,(rebvio::types::Float)_msg->angular_velocity.y,(rebvio::types::Float)_msg->angular_velocity.z),
		TooN::makeVector((rebvio::types::Float)_msg->linear_acceleration.x,(rebvio::types::Float)_msg->linear_acceleration.y,(rebvio::types::Float)_msg->linear_acceleration.z)});
}

void RosRebvio::registerOdometryCallback(std::function<void(rebvio::Rebvio::Odometry&)> _cb) {
  rebvio_.registerOdometryCallback(_cb);
}

bool RosRebvio::run(std::string _bag_file) {
	// Run live node in case bag_file parameter is not specified
	if(_bag_file.empty()) {
		ROS_INFO_STREAM("Running with published input data.");
		ros::spin();
	} else {
		ROS_INFO_STREAM("Running with rosbag file: "<<_bag_file);
		rosbag::Bag bag;
		bag.open(_bag_file,rosbag::bagmode::Read);
		ros::Time last_bagtime = rosbag::View(bag).getBeginTime(); // use image timestamps to simulate realtime playback
		double start_time,end_time;
		ros::Time starttime = ros::TIME_MIN;
		ros::Time endtime = ros::TIME_MAX;
		if(nh_.getParam("start_time",start_time)) {
			starttime = rosbag::View(bag).getBeginTime()+ros::Duration(start_time);
			last_bagtime = starttime;
		}
		if(nh_.getParam("end_time",end_time)) endtime = rosbag::View(bag).getBeginTime()+ros::Duration(end_time);
		ros::Time last_realtime = ros::Time::now();
		for(rosbag::MessageInstance const m : rosbag::View(bag,starttime,endtime)) {
			if(!ros::ok()) break;
			if(m.getTopic() == imu_topic_) {
				sensor_msgs::ImuConstPtr imu_msg = m.instantiate<sensor_msgs::Imu>();
				if(imu_msg) imuCallback(imu_msg);
			} else if(m.getTopic() == cam_topic_) {
				sensor_msgs::ImageConstPtr image_msg = m.instantiate<sensor_msgs::Image>();
				ros::Duration dt = (image_msg->header.stamp-last_bagtime)-(ros::Time::now()-last_realtime);
				if(dt.toSec() > 0.0) ros::Duration(dt.toSec()*2.0).sleep();
				if(image_msg) imageCallback(image_msg);
				last_bagtime = image_msg->header.stamp;
				last_realtime = ros::Time::now();
			}
		}
		bag.close();
		ROS_INFO_STREAM("Finished rosbag playback.");
	}
	return true;
}



}

