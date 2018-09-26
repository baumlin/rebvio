/*
 * primitives.hpp
 *
 *  Created on: Sep 2, 2018
 *      _inuthor: baumlin
 */

#ifndef INCLUDE_REBVIO_TYPES_PRIMITIVES_HPP_
#define INCLUDE_REBVIO_TYPES_PRIMITIVES_HPP_

#include <TooN/TooN.h>

namespace rebvio {
namespace types {

struct Vector2 {
	float x;
	float y;
};

struct Vector3 {
	float x;
	float y;
	float z;
};

using Vector2f = TooN::Vector<2,float>;
using Vector3f = TooN::Vector<3,float>;
using Vector6f = TooN::Vector<6,float>;
using Vector7f = TooN::Vector<7,float>;
using Vector11f = TooN::Vector<11,float>;
using Matrix3f = TooN::Matrix<3,3,float>;
using Matrix6f = TooN::Matrix<6,6,float>;
using Matrix7f = TooN::Matrix<7,7,float>;
using Matrix11f = TooN::Matrix<11,11,float>;


/**
 * Inverts a 3x3 Matrix (not available as basic functionality in TooN)
 *
 * 			 | a b c |												| ei-fh ch-bi bf-ce |
 * _in = | d e f | -> _in^-1 = 1/det(_in)*| fg-di ai-cg cd-af |
 * 			 | g h i |												| dh-eg bg-ah ae-bd |
 */
inline Matrix3f invert(const rebvio::types::Matrix3f& _in) {

	Matrix3f out;
  out(0,0) = _in(1,1)*_in(2,2)-_in(1,2)*_in(2,1); // ei-fh
  out(0,1) = _in(0,2)*_in(2,1)-_in(0,1)*_in(2,2); // ch-bi
  out(0,2) = _in(0,1)*_in(1,2)-_in(0,2)*_in(1,1); // bf-ce
  out(1,0) = _in(1,2)*_in(2,0)-_in(1,0)*_in(2,2); // fg-di
  out(1,1) = _in(0,0)*_in(2,2)-_in(0,2)*_in(2,0); // ai-cg
  out(1,2) = _in(0,2)*_in(1,0)-_in(0,0)*_in(1,2); // cd-af
  out(2,0) = _in(1,0)*_in(2,1)-_in(1,1)*_in(2,0); // dh-eg
  out(2,1) = _in(0,1)*_in(2,0)-_in(0,0)*_in(2,1); // bg-ah
  out(2,2) = _in(0,0)*_in(1,1)-_in(0,1)*_in(1,0); // ae-bd
  return out/TooN::determinant(_in);
}

} /* namespace types */
} /* namespace rebvio */



#endif /* INCLUDE_REBVIO_TYPES_PRIMITIVES_HPP_ */
