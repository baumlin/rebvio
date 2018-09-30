/*
 * image.hpp
 *
 *  Created on: Sep 1, 2018
 *      Author: baumlin
 */

#ifndef INCLUDE_REBVIO_IMAGE_DATA_HPP_
#define INCLUDE_REBVIO_IMAGE_DATA_HPP_

#include <opencv2/core.hpp>

namespace rebvio {
namespace types {

struct Image {
	uint64_t ts_us;	//!< Timestamp in [us]
	cv::Mat data;		//!< Image Data
};

} /* namespace types */
} /* namespace rebvio */

#endif /* INCLUDE_REBVIO_IMAGE_DATA_HPP_ */
