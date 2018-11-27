/*
 * odometry.hpp
 *
 *  Created on: Sep 1, 2018
 *      Author: baumlin
 */

#ifndef INCLUDE_REBVIO_ODOMETRY_HPP_
#define INCLUDE_REBVIO_ODOMETRY_HPP_

#include "rebvio/types/definitions.hpp"

namespace rebvio {
namespace types {

/**
 * \brief Struct holding a stamped Odometry measurement
 */
struct Odometry {
	uint64_t ts_us;                       //!< Timestamp in [us]
	rebvio::types::Vector3f orientation;  //!< Orientation (lie algebra)
	rebvio::types::Vector3f position;     //!< Position
};

} /* namespace types */
} /* namespace rebvio */

#endif /* INCLUDE_REBVIO_ODOMETRY_HPP_ */
