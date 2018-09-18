/*
 * imu.hpp
 *
 *  Created on: Sep 1, 2018
 *      Author: baumlin
 */

#ifndef INCLUDE_REBVIO_IMU_DATA_HPP_
#define INCLUDE_REBVIO_IMU_DATA_HPP_

#include "rebvio/types/primitives.hpp"
#include "rebvio/camera.hpp"
#include <eigen3/Eigen/Geometry>
#include <cmath>

namespace rebvio {
namespace types {

struct Imu {
	uint64_t ts;
	Eigen::Vector3f gyro;
	Eigen::Vector3f acc;
};

struct IntegratedImu {
public:
	IntegratedImu() :
		n_(0),
		last_ts_(0),
		R_(Eigen::Quaternionf::Identity()),
		gyro_(Eigen::Vector3f::Zero()),
		acc_(Eigen::Vector3f::Zero()),
		dgyro_(Eigen::Vector3f::Zero()),
		cacc_(Eigen::Vector3f::Zero()) {}

	void add(rebvio::types::Imu& _imu, const Eigen::Matrix3f& _R_c2i, float& _dt) {
		++n_;
		Eigen::Vector3f tmp = _R_c2i.transpose()*_imu.gyro;
		gyro_ += tmp;
		acc_ += _R_c2i.transpose()*_imu.acc;
		R_ = R_*IntegratedImu::expMap(tmp*_dt);
	}

	const IntegratedImu& get() {
		if(n_ > 1) {
			gyro_ /= float(n_);
			acc_ /= float(n_);
			// TODO: dgyro

			n_ = 0;
		}
		// TODO: cacc
		return *this;
	}

	Eigen::Matrix3f getR() const {
		return R_.toRotationMatrix();
	}

private:
	static Eigen::Quaternionf expMap(const Eigen::Vector3f& _theta) {
		const float theta_squared_norm = _theta.squaredNorm();
		if(theta_squared_norm < 1e-6) {
			Eigen::Quaternionf q(1,_theta(0)*0.5,_theta(1)*0.5,_theta(2)*0.5);
			q.normalize();
			return q;
		}
		const float theta_norm = sqrt(theta_squared_norm);
		const Eigen::Vector3f q_imag = sin(theta_norm*0.5)*_theta/theta_norm;
		Eigen::Quaternionf q(cos(theta_norm*0.5),q_imag(0),q_imag(1),q_imag(2));
		return q;
	}

private:
	unsigned int n_;
	uint64_t last_ts_;
	Eigen::Quaternionf R_;
	Eigen::Vector3f gyro_;
	Eigen::Vector3f acc_;
	Eigen::Vector3f dgyro_;
	Eigen::Vector3f cacc_;
};

} /* namespace types */
} /* namespace rebvio */

#endif /* INCLUDE_REBVIO_IMU_DATA_HPP_ */
