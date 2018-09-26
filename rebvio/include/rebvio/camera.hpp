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
#include "rebvio/types/primitives.hpp"

namespace rebvio {

/**
 * @brief Class that implements the Pinhole camera model
 */
class Camera {
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

//	inline TooN::Vector<2> normalize(const TooN::Vector<2>& _p) const {
//		return TooN::makeVector(float((_p[0]-cx_)/fx_),float((_p[1]-cy_)/fy_));
//	}

	inline rebvio::types::Vector3f normalize(const rebvio::types::Vector2f& _p) const {
		return TooN::makeVector(float((_p[0]-cx_)/fm_),float((_p[1]-cy_)/fm_),1.0);
	}

public:
	float fx_;
	float fy_;
	float fm_; //!< mean focal length
	float cx_;
	float cy_;
	float k1_;
	float k2_;
	float k3_;
	float p1_;
	float p2_;
	unsigned int rows_;
	unsigned int cols_;

private:
//	Eigen::Matrix3f R_c2i_;
//	Eigen::Vector3f t_c2i_;
	rebvio::types::Matrix3f R_c2i_;
	rebvio::types::Vector3f t_c2i_;

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef std::shared_ptr<rebvio::Camera> CameraPtr;

} /* namespace rebvio */



#endif /* INCLUDE_REBVIO_CAMERA_HPP_ */
