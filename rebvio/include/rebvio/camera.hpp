/*
 * keyline.hpp
 *
 *  Created on: Aug 29, 2018
 *      Author: baumlin
 */

#ifndef INCLUDE_REBVIO_CAMERA_HPP_
#define INCLUDE_REBVIO_CAMERA_HPP_

#include <eigen3/Eigen/Dense>
#include <memory>

namespace rebvio {

class Camera {
public:
	Camera() :
		rows_(480),
		cols_(752),
		R_c2i_(Eigen::Matrix3f::Identity()),
		t_c2i_(Eigen::Vector3f::Zero())
		{}
	const Eigen::Matrix3f& getRc2i() const {
		return R_c2i_;
	}

public:
	unsigned int rows_;
	unsigned int cols_;

private:
	Eigen::Matrix3f R_c2i_;
	Eigen::Vector3f t_c2i_;

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef std::shared_ptr<rebvio::Camera> CameraPtr;

} /* namespace rebvio */



#endif /* INCLUDE_REBVIO_CAMERA_HPP_ */
