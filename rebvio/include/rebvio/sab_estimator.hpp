/*
 * sab_estimator.hpp
 *
 *  Created on: Oct 27, 2018
 *      Author: baumlin
 */

#ifndef INCLUDE_REBVIO_SAB_ESTIMATOR_HPP_
#define INCLUDE_REBVIO_SAB_ESTIMATOR_HPP_

#include "rebvio/types/definitions.hpp"

namespace rebvio {

class SABEstimator {
public:
	struct Config {
		rebvio::types::Vector3f a_v;    //!< Visual acceleration
		rebvio::types::Vector3f a_s;    //!< Gravity-corrected acceleration
		rebvio::types::Float G;                 //!< Standard gravity (norm of gravitational acceleration)
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

public:
	SABEstimator(SABEstimator::Config& _config);
	SABEstimator() = delete;
	~SABEstimator();
	bool problem(rebvio::types::Matrix7f& _JtJ, rebvio::types::Vector7f& _JtF, const rebvio::types::Vector7f& _X);
	int gaussNewton(rebvio::types::Vector7f& _X, int _iter_max, rebvio::types::Float _a_tol = 0.0, rebvio::types::Float _r_tol = 0.0);

private:
	inline rebvio::types::Float saturate(rebvio::types::Float _t, rebvio::types::Float _limit);

private:
	SABEstimator::Config config_;



};

} /* namespace rebvio */

#endif /* INCLUDE_REBVIO_SAB_ESTIMATOR_HPP_ */
