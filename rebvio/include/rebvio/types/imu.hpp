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
#include <cmath>
#include <limits.h>

#include <TooN/so3.h>

namespace rebvio {
namespace types {

struct Imu {
	uint64_t ts;					//!< Timestamp in [us]
	rebvio::types::Vector3f gyro;
	rebvio::types::Vector3f acc;

};

class IntegratedImu {
public:
	IntegratedImu() :
		n_(0),
		init_ts_(0),
		last_ts_(0),
		dt_(0),
		R_(TooN::Identity),
		gyro_(TooN::Zeros),
		acc_(TooN::Zeros),
		dgyro_(TooN::Zeros),
		cacc_(TooN::Zeros) {}

	void add(rebvio::types::Imu& _imu, const rebvio::types::Matrix3f& _R_c2i) {
		++n_;
		rebvio::types::Vector3f tmp = _R_c2i.T()*_imu.gyro;
		gyro_ += tmp;
		acc_ += _R_c2i.T()*_imu.acc;
		float dt;
		if(last_ts_ == 0) {
			init_ts_ = _imu.ts;
			last_ts_ = init_ts_;
			dt = 0.0;
			gyro_init_ = _imu.gyro;
			gyro_last_ = gyro_init_;
		} else {
			dt = float(_imu.ts-last_ts_)/1000000.0; // convert to [s]
		}
		R_ = R_*TooN::SO3<float>(tmp*dt).get_matrix();
		last_ts_ = _imu.ts;
		gyro_last_ = _imu.gyro;
	}

	const IntegratedImu& get(const rebvio::types::Matrix3f& _R_c2i, const rebvio::types::Vector3f _t_c2i) {
		dt_ = (last_ts_-init_ts_)/(n_-1)*n_; //estimate sampling time from interval and extrapolate to include initial measurement
		if(n_ > 1) {
			gyro_ /= float(n_);
			acc_ /= float(n_);

			dgyro_ = _R_c2i.T()*(gyro_last_-gyro_init_)/dt_s();
		}
		cacc_ = acc_+(dgyro_^(-(_R_c2i.T()*_t_c2i)));
		n_ = 0;
		init_ts_ = 0;
		last_ts_ = 0;
		return *this;
	}

	const uint64_t& dt_us() const {
		return dt_;
	}

	float dt_s() const {
		return float(dt_)/1000000.0;
	}

	const rebvio::types::Vector3f& gyro() const {
		return gyro_;
	}

	const rebvio::types::Vector3f& acc() const {
		return acc_;
	}

	const rebvio::types::Vector3f& cacc() const {
		return cacc_;
	}

	const rebvio::types::Matrix3f& R() const {
		return R_;
	}

private:
	unsigned int n_;
	uint64_t last_ts_;				//!< Timestamp of last measurement [us]
	uint64_t init_ts_; 				//!< Timestamp of first measurement [us]
	uint64_t dt_;							//!< Duration of integration interval in [us]

	rebvio::types::Matrix3f R_;	//!< Interframe rotation
	rebvio::types::Vector3f gyro_;	  //!< Mean gyro measurement
	rebvio::types::Vector3f gyro_init_;	  //!< First gyro measurement
	rebvio::types::Vector3f gyro_last_;	  //!< Last gyro measurement
	rebvio::types::Vector3f acc_;		//!< Mean accelerometer measurement
	rebvio::types::Vector3f dgyro_;	//!< Mean gyro angular acceleration
	rebvio::types::Vector3f cacc_;		//!< Compensated acceleration
};

struct ImuState {
	struct ImuStateConfig {
		float g_module{9.81}; //!< Measured gravity module
		float g_uncertainty{2e-3}; //!< Process uncertainty on the g vector
		float g_module_uncertainty{0.2e3}; //!< Uncertainty in the g module: keep at big value
		float acc_std_dev{2.0e-3}; //!< Accelerometer noise std dev
		float gyro_std_dev{1.6968e-04}; //!< Gyro noise std dev
		float gyro_bias_std_dev{1.9393e-05}; //!< Gyro bias random walk noise
		float vbias_std_dev{1e-7}; //!< Process uncertainty in the visual bias estimation: keep lower than the gyro_bias_std_dev
		float scale_std_dev_mult{1e-2}; //!< Scale process uncertainty in relation to visual: use this parameter to tune filter response
		float scale_std_dev_max{1e-4}; //!< Max scale process uncertainty
		float scale_stdd_dev_init{1.2e-3}; //!< Initial scale process uncertainty
		int init_bias{1}; //!< 0: use initial guess, 1: use init_bias_frame_num frames to estimate bias
		int init_bias_frame_num{10};
		rebvio::types::Vector3f init_bias_guess{TooN::makeVector(0.0188, 0.0037, 0.0776)}; //!< Initial bias guess in camera frame
	};

	rebvio::types::Vector3f Vg{TooN::Zeros}; //!< IMU Stages Velocity and Rotation

	rebvio::types::Vector3f dVv{TooN::Zeros};
	rebvio::types::Vector3f dWv{TooN::Zeros};

	rebvio::types::Vector3f dVgv{TooN::Zeros};
	rebvio::types::Vector3f dWgv{TooN::Zeros};

	rebvio::types::Vector3f Vgv{TooN::Zeros};
	rebvio::types::Vector3f Wgv{TooN::Zeros};

	rebvio::types::Vector3f dVgva{TooN::Zeros};
	rebvio::types::Vector3f dWgva{TooN::Zeros};
	rebvio::types::Vector3f Vgva{TooN::Zeros};

	rebvio::types::Matrix3f P_Vg{TooN::Identity*std::numeric_limits<float>::max()}; //!< IMU Stages Velocity and Rotation Covariances

	rebvio::types::Matrix3f RGyro{TooN::Identity};
	rebvio::types::Matrix3f RGBias{TooN::Identity};

	rebvio::types::Vector3f Bg{TooN::Zeros};						//!< Gyro bias
	rebvio::types::Matrix3f W_Bg{TooN::Identity*std::numeric_limits<float>::max()}; //!< Gyro bias covariance

	rebvio::types::Vector3f Av{TooN::Zeros};						//!< Visual acceleration
	rebvio::types::Vector3f As{TooN::Zeros};						//!< Accelerometer acceleration

	TooN::Vector<7,float> X;
	TooN::Matrix<7,7,float> P;
	rebvio::types::Matrix3f Qrot;
	rebvio::types::Matrix3f Qg;
	rebvio::types::Matrix3f Qbias;

	float QKp;
	float Rg;

	rebvio::types::Matrix3f Rs;
	rebvio::types::Matrix3f Rv;
	rebvio::types::Vector3f g_est;
	rebvio::types::Vector3f u_est;
	rebvio::types::Vector3f b_est;

	rebvio::types::Matrix3f Wvw;
	TooN::Vector<6,float> Xvw;

	rebvio::types::Vector3f Posgv{TooN::Zeros};
	rebvio::types::Vector3f Posgva{TooN::Zeros};

	bool initialized{false};

	ImuState(ImuStateConfig& _config) {
		W_Bg = types::invert(100.0*RGBias);
		Qg = TooN::Identity*_config.g_uncertainty*_config.g_uncertainty;
		Rg = _config.g_module_uncertainty*_config.g_module_uncertainty;
		Rs = TooN::Identity*_config.acc_std_dev*_config.acc_std_dev;
		Qbias = TooN::Identity*_config.vbias_std_dev*_config.vbias_std_dev;
		X  = TooN::makeVector(M_PI_4,0.0,_config.g_module,0.0,0.0,0.0,0.0);
		P = TooN::makeVector(_config.scale_stdd_dev_init*_config.scale_stdd_dev_init,
				100.0,
				100.0,
				100.0,
				_config.vbias_std_dev*_config.vbias_std_dev*1e1,
				_config.vbias_std_dev*_config.vbias_std_dev*1e1,
				_config.vbias_std_dev*_config.vbias_std_dev*1e1).as_diagonal();
		u_est  = TooN::makeVector(1.0, 0.0, 0.0);
	}

};

} /* namespace types */
} /* namespace rebvio */

#endif /* INCLUDE_REBVIO_IMU_DATA_HPP_ */
