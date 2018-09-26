/*
 * rebvio_params.hpp
 *
 *  Created on: Aug 29, 2018
 *      Author: baumlin
 */

#ifndef INCLUDE_REBVIO_PARAMS_HPP_
#define INCLUDE_REBVIO_PARAMS_HPP_

#include "rebvio/types/imu.hpp"

namespace rebvio {

struct RebvioParams {



	// IMU
	rebvio::types::ImuState::ImuStateConfig imu_state_config_;

};

} /* namespace rebvio */

#endif /* INCLUDE_REBVIO_PARAMS_HPP_ */
