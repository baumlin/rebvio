/*
 * imu.hpp
 *
 *  Created on: Sep 1, 2018
 *      Author: baumlin
 */

#ifndef INCLUDE_REBVIO_IMU_DATA_HPP_
#define INCLUDE_REBVIO_IMU_DATA_HPP_

#include "rebvio/camera.hpp"
#include <cmath>
#include <limits.h>
#include <rebvio/types/definitions.hpp>

#include <TooN/so3.h>

namespace rebvio {
namespace types {

/**
 * \brief Struct holding a stamped IMU measurement
 */
struct Imu {
	uint64_t ts;                    //!< Timestamp in [us]
	rebvio::types::Vector3f gyro;   //!< Gyroscope measurement in [rad/s]
	rebvio::types::Vector3f acc;    //!< Accelerometer measurement in [m/s^2]

};

/**
 * \brief Class for IMU measurement integration
 */

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

	/**
	 * \brief Adds a new IMU measurement to the Integrator, without actually performing the integration (see get() method)
	 */
	void add(rebvio::types::Imu& _imu, const rebvio::types::Matrix3f& _R_c2i) {
		rebvio::types::Vector3f tmp = _R_c2i.T()*_imu.gyro;
		types::Float dt;
		if(last_ts_ == 0) {
			n_ = 1;
			init_ts_ = _imu.ts;
			last_ts_ = init_ts_;
			dt = 0.005;			// TODO: don't use hardcoded value
			gyro_init_ = _imu.gyro;
			gyro_last_ = gyro_init_;
			R_ = TooN::Identity;
			gyro_ = tmp;
			acc_ = _R_c2i.T()*_imu.acc;
			dgyro_ = TooN::Zeros;
			cacc_ = TooN::Zeros;
		} else {
			++n_;
			dt = types::Float(_imu.ts-last_ts_)/1000000.0; // convert to [s]
			gyro_ += tmp;
			acc_ += _R_c2i.T()*_imu.acc;
		}
		R_ = R_*TooN::SO3<types::Float>(tmp*dt).get_matrix();
		last_ts_ = _imu.ts;
		gyro_last_ = _imu.gyro;
	}

	/**
	 * \brief Calculates and returns the integrated IMU measurements. Only call this once for an integration interval! Otherwise use the getters
	 */
	const IntegratedImu& get(const rebvio::types::Matrix3f& _R_c2i, const rebvio::types::Vector3f _t_c2i) {
		dt_ = (last_ts_-init_ts_)/(n_-1)*n_; // estimate sampling time from interval and extrapolate to include initial measurement
		if(n_ > 1) {
			gyro_ /= types::Float(n_);
			acc_ /= types::Float(n_);
			dgyro_ = _R_c2i.T()*(gyro_last_-gyro_init_)/dt_s();
		}

		cacc_ = acc_+(dgyro_^(-(_R_c2i.T()*_t_c2i)));
		n_ = 0;
		init_ts_ = 0;
		last_ts_ = 0;

		return *this;
	}

	/**
	 * \brief Returns the integration interval in [us]
	 */
	const uint64_t& dt_us() const {
		return dt_;
	}

	/**
	 * \brief Returns the integration interval in [s]
	 */
	types::Float dt_s() const {
		return types::Float(dt_)/1000000.0;
	}

	/**
	 * \brief Returns the estimated gyro measurement
	 */
	const rebvio::types::Vector3f& gyro() const {
		return gyro_;
	}

	/**
	 * \brief Returns the estimated accelerometer measurement
	 */
	const rebvio::types::Vector3f& acc() const {
		return acc_;
	}

	/**
	 * \brief Returns the estimated compensated accelerometer measurement
	 */
	const rebvio::types::Vector3f& cacc() const {
		return cacc_;
	}

	/**
	 * \brief Returns the estimated Rotation
	 */
	const rebvio::types::Matrix3f& R() const {
		return R_;
	}

private:
	unsigned int n_;                     //!< Number of IMU measurements used for integration
	uint64_t last_ts_;                   //!< Timestamp of last measurement [us]
	uint64_t init_ts_;                   //!< Timestamp of first measurement [us]
	uint64_t dt_;                        //!< IMU integration interval in [us]

	rebvio::types::Matrix3f R_;          //!< Interframe rotation (between first and last measurement in integration interval)
	rebvio::types::Vector3f gyro_;       //!< Mean gyro measurement
	rebvio::types::Vector3f gyro_init_;  //!< First gyro measurement
	rebvio::types::Vector3f gyro_last_;  //!< Last gyro measurement
	rebvio::types::Vector3f acc_;        //!< Mean accelerometer measurement
	rebvio::types::Vector3f dgyro_;      //!< Mean gyro angular acceleration
	rebvio::types::Vector3f cacc_;       //!< Compensated acceleration
};


struct ImuStateConfig {
	types::Float g_norm{9.81};                   //!< Measured gravity norm
	types::Float g_uncertainty{2e-3};            //!< Process uncertainty on the g vector
	types::Float g_norm_uncertainty{0.2e3};      //!< Uncertainty in the g norm: keep at big value
	types::Float acc_std_dev{2.0e-3};            //!< Accelerometer noise std dev
	types::Float gyro_std_dev{1.6968e-04};       //!< Gyro noise std dev
	types::Float gyro_bias_std_dev{1.9393e-05};  //!< Gyro bias random walk noise
	types::Float vbias_std_dev{1e-7};            //!< Process uncertainty in the visual bias estimation: keep lower than the gyro_bias_std_dev
	types::Float scale_std_dev_mult{1e-2};       //!< Scale process uncertainty in relation to visual: use this parameter to tune filter response
	types::Float scale_std_dev_max{1e-4};        //!< Max scale process uncertainty
	types::Float scale_stdd_dev_init{1.2e-3};    //!< Initial scale process uncertainty
	int init_bias{1};                            //!< 0: use initial guess, 1: use init_bias_frame_num frames to estimate bias
	int init_bias_frame_num{10};                 //!< Number of frames to estimate the bias
	rebvio::types::Vector3f init_bias_guess{TooN::makeVector(0.0188, 0.0037, 0.0776)}; //!< Initial bias guess in camera frame
};

struct ImuState {
	rebvio::types::Vector3f Vg{TooN::Zeros};                        //!< Inter-frame translation from gyro prior
	rebvio::types::Matrix3f P_Vg{TooN::Identity*std::numeric_limits<types::Float>::max()}; //!< Inter-frame translation Covariance from gyro prior
	rebvio::types::Vector3f Vgv{TooN::Zeros};				            		//!< Inter-frame translation from gyro and visual input
	rebvio::types::Vector3f dVgv{TooN::Zeros};                      //!< Correction of inter-frame translation from visual input
	rebvio::types::Vector3f dWgv{TooN::Zeros};                      //!< Correction of inter-frame rotation from visual input
	rebvio::types::Vector3f Vgva{TooN::Zeros};                      //!< Inter-frame translation from gyro, visual and accelerometer input
	rebvio::types::Vector3f dVgva{TooN::Zeros};                     //!< Correction of inter-frame translation from accelerometer input
	rebvio::types::Vector3f dWgva{TooN::Zeros};                     //!< Correction of inter-frame rotation from accelerometer input
	rebvio::types::Vector3f Bg{TooN::Zeros};                        //!< Gyro bias
	rebvio::types::Matrix3f RGBias{TooN::Identity};                 //!< Gyro bias covariance
	rebvio::types::Matrix3f W_Bg{types::invert(100.0*RGBias)};       //!< Gyro bias information matrix
	rebvio::types::Matrix3f RGyro{TooN::Identity};                  //!< Gyro measurement covariance
	rebvio::types::Vector3f Av{TooN::Zeros};                        //!< Visual acceleration (calculated via numerical differentiation of the velocity estimate)
	rebvio::types::Vector3f As{TooN::Zeros};                        //!< Gravity-corrected acceleration
	rebvio::types::Vector3f u_est{TooN::makeVector(1.0, 0.0, 0.0)}; //!<
	bool initialized{false};                                        //!< State Initialization Flag
};

} /* namespace types */
} /* namespace rebvio */

#endif /* INCLUDE_REBVIO_IMU_DATA_HPP_ */
