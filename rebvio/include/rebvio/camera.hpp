/*
 * camera.hpp
 *
 *  Created on: Aug 29, 2018
 *      Author: baumlin
 */

#ifndef INCLUDE_REBVIO_CAMERA_HPP_
#define INCLUDE_REBVIO_CAMERA_HPP_

#include "rebvio/types/definitions.hpp"
#include <memory>
#include <opencv2/imgproc.hpp>

namespace rebvio {

/**
 * \brief Class that implements the Pinhole camera model with extrinsics to an optional IMU
 */
class Camera {
public:
	using SharedPtr = std::shared_ptr<rebvio::Camera>;

public:
	Camera() :
		fx_(458.654	),
		fy_(457.296),
		fm_(0.5*(fx_+fy_)),
		cx_(367.215),
		cy_(248.375),
		k1_(-0.28340811),
		k2_(0.07395907),
		k3_(0.0),
		p1_(0.00019359),
		p2_(1.76187114e-05),
		rows_(480),
		cols_(752)
		{
		  K_ = (cv::Mat_<types::Float>(3,3) << fm_,0.0,cx_,0.0,fm_,cy_,0.0,0.0,1.0);
		  D_ = (cv::Mat_<types::Float>(1,5) << k1_,k2_,p1_,p2_,k3_);
			R_c2i_ = TooN::Data(0.0148655429818, -0.999880929698, 0.00414029679422,
          0.999557249008, 0.0149672133247, 0.025715529948,
          -0.0257744366974, 0.00375618835797, 0.999660727178);
			t_c2i_ = TooN::makeVector(-0.0216401454975, -0.064676986768, 0.00981073058949);
		}
	const rebvio::types::Matrix3f& getRc2i() const {
		return R_c2i_;
	}

	const rebvio::types::Vector3f& getTc2i() const {
		return t_c2i_;
	}

	cv::Mat undistort(cv::Mat& _in) {
		cv::Mat out;
		cv::undistort(_in,out,K_,D_);
		return out;
	}

public:
	types::Float fx_;		//!< Focal length x component
	types::Float fy_;		//!< Focal length y component
	types::Float fm_;		//!< Mean focal length
	types::Float cx_;		//!< Principal point x component
	types::Float cy_;		//!< Principal point y component
	types::Float k1_;		//!< First radial distortion coefficient in the Rad-Tan distortion model
	types::Float k2_;		//!< Second radial distortion coefficient in the Rad-Tan distortion model
	types::Float k3_;		//!< Third radial distortion coefficient in the Rad-Tan distortion model
	types::Float p1_;		//!< First tangential distortion coefficient in the Rad-Tan distortion model
	types::Float p2_;		//!< Second tangential distortion coefficient in the Rad-Tan distortion model
	unsigned int rows_;	//!< y-resolution
	unsigned int cols_;	//!< x-resolution

private:
	cv::Mat K_;                     //!< Camera Matrix for OpenCV Undistortion
	cv::Mat D_;                     //!< Distortion Parameters for OpenCV Undistortion
	rebvio::types::Matrix3f R_c2i_;	//!< Rotation component of transformation from Camera to Imu frame
	rebvio::types::Vector3f t_c2i_;	//!< Translation component of transformation from Camera to Imu frame
};


} /* namespace rebvio */



#endif /* INCLUDE_REBVIO_CAMERA_HPP_ */
