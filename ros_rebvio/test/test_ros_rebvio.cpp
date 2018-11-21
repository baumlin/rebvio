#include <gtest/gtest.h>
#include "ros_rebvio/ros_rebvio.hpp"

#include <ros/ros.h>
#include <ros/package.h>
#include <functional>
#include <fstream>



TEST(RegressionTest, testOdometry) {
	ros::NodeHandle nh;
	ros::NodeHandle nhp("~");

	ROS_INFO_STREAM("Setting up "<<ros::this_node::getName()<<" Node");

	rebvio::RosRebvio rebvio(nhp);

	std::vector<rebvio::Rebvio::Odometry> odometry_result;
	std::function<void(rebvio::Rebvio::Odometry&)> odometry_cb = [&](rebvio::Rebvio::Odometry& _msg) {
    odometry_result.push_back(_msg);
	};
	rebvio.registerOdometryCallback(odometry_cb);

	std::string package_path = ros::package::getPath("ros_rebvio");
	std::string bag_file = package_path+"/test/data/MH_03_medium_test_15s-30s.bag";
	rebvio.run(bag_file);

	std::string odometry_file = package_path+"/test/data/MH_03_medium_test_15s-30s_odometry.txt";
	std::ifstream file(odometry_file);
  std::string line;
  int i = 0;
  while(std::getline(file,line)) {
  	if(i >= odometry_result.size()) break;
  	std::istringstream iss(line);
  	std::vector<std::string> odometry{std::istream_iterator<std::string>{iss},std::istream_iterator<std::string>{}};
  	EXPECT_EQ(std::stoull(odometry[0]),odometry_result[i].ts_us);
  	EXPECT_NEAR(std::stof(odometry[1]),odometry_result[i].orientation[0],1e-6);
  	EXPECT_NEAR(std::stof(odometry[2]),odometry_result[i].orientation[1],1e-6);
  	EXPECT_NEAR(std::stof(odometry[3]),odometry_result[i].orientation[2],1e-6);
  	EXPECT_NEAR(std::stof(odometry[4]),odometry_result[i].position[0],1e-6);
  	EXPECT_NEAR(std::stof(odometry[5]),odometry_result[i].position[1],1e-6);
  	EXPECT_NEAR(std::stof(odometry[6]),odometry_result[i].position[2],1e-6);
  	++i;
  }
}


int main(int argc, char** argv) {
	ros::init(argc,argv,"RosRebvioTest");
	testing::InitGoogleTest(&argc,argv);
	auto res = RUN_ALL_TESTS();
	ros::shutdown();
	return res;
}
