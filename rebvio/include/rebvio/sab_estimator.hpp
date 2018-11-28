/*
 * sab_estimator.hpp
 *
 *  Created on: Oct 27, 2018
 *      Author: baumlin
 */

#ifndef INCLUDE_REBVIO_SAB_ESTIMATOR_HPP_
#define INCLUDE_REBVIO_SAB_ESTIMATOR_HPP_

#include "rebvio/types/definitions.hpp"
#include "rebvio/types/imu.hpp"

namespace rebvio {

/**
 * \class Scale-Attitude-Bias Estimator
 */
class SABEstimator {
public:
	struct Config {
		rebvio::types::Vector3f a_v;    //!< Visual acceleration
		rebvio::types::Vector3f a_s;    //!< Gravity-corrected acceleration
		rebvio::types::Float G;         //!< Standard gravity (norm of gravitational acceleration)
		rebvio::types::Vector7f x_p;    //!< Prior filter state
		rebvio::types::Matrix7f Pp;     //!< Prior filter state covariance
		rebvio::types::Matrix3f Rv;     //!< Observation noise of visual acceleration
		rebvio::types::Matrix3f Rs;     //!< Observation noise of gravity-corrected acceleration
		rebvio::types::Float Rg;                //!< Observation noise of standard gravity

		Config(const rebvio::types::Vector3f& _a_v, const rebvio::types::Vector3f& _a_s, rebvio::types::Float _G, const rebvio::types::Vector7f& _x_p,
					 const rebvio::types::Matrix3f& _Rv, const rebvio::types::Matrix3f& _Rs, rebvio::types::Float _Rg, const rebvio::types::Matrix7f& _Pp) :
						 a_v(_a_v), a_s(_a_s), G(_G), x_p(_x_p), Rv(_Rv), Rs(_Rs), Rg(_Rg), Pp(_Pp) {}
		Config() = delete;
	};

	struct State {
		rebvio::types::Vector7f X;      //!< State of the filter: X = [g,a,b] with g: estimated gravity vector, a: angle scale, b: visual rotation bias vector
		rebvio::types::Vector3f g_est;  //!< Estimated gravity state of the  filter
		rebvio::types::Vector3f b_est;  //!< Estimated visual rotation bias state of the filter
		rebvio::types::Matrix7f P;      //!< State covariance matrix of the filter
		rebvio::types::Matrix3f Qrot;   //!< Process noise of the visual rotation
		rebvio::types::Matrix3f Qg;     //!< Process noise of gravity vector state
		rebvio::types::Matrix3f Qbias;  //!< Process noise of visual rotation bias vector state
		types::Float QKp;               //!< Process noise of angle scale state
		types::Float Rg;                //!< Observation noise of the standard gravity (norm of the gravitational acceleration)
		rebvio::types::Matrix3f Rs;     //!< Observation noise of the gravity-corrected acceleration
		rebvio::types::Matrix3f Rv;     //!< Observation noise of the visual acceleration
		State(rebvio::types::ImuStateConfig& _config) {
			Qg = TooN::Identity*_config.g_uncertainty*_config.g_uncertainty;
			Rg = _config.g_norm_uncertainty*_config.g_norm_uncertainty;
			Rs = TooN::Identity*_config.acc_std_dev*_config.acc_std_dev;
			Qbias = TooN::Identity*_config.vbias_std_dev*_config.vbias_std_dev;
			X  = TooN::makeVector(M_PI_4,0.0,_config.g_norm,0.0,0.0,0.0,0.0);
			P = TooN::makeVector(_config.scale_stdd_dev_init*_config.scale_stdd_dev_init,
					100.0,
					100.0,
					100.0,
					_config.vbias_std_dev*_config.vbias_std_dev*1e1,
					_config.vbias_std_dev*_config.vbias_std_dev*1e1,
					_config.vbias_std_dev*_config.vbias_std_dev*1e1).as_diagonal();

		}
	};


public:
	SABEstimator(SABEstimator::Config& _config);
	SABEstimator() = delete;
	~SABEstimator();
	bool problem(rebvio::types::Matrix7f& _JtJ, rebvio::types::Vector7f& _JtF, const rebvio::types::Vector7f& _X);
	int gaussNewton(rebvio::types::Vector7f& _X, int _iter_max, rebvio::types::Float _a_tol = 0.0, rebvio::types::Float _r_tol = 0.0);

private:
	inline rebvio::types::Float saturate(rebvio::types::Float _t, rebvio::types::Float _limit);

private:
	SABEstimator::Config config_;  //!< Estimator Configuration



};

} /* namespace rebvio */

#endif /* INCLUDE_REBVIO_SAB_ESTIMATOR_HPP_ */
