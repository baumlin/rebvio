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

typedef TooN::Vector<2,float> Vector2f;
typedef TooN::Vector<3,float> Vector3f;
typedef TooN::Vector<6,float> Vector6f;
typedef TooN::Vector<7,float> Vector7f;
typedef TooN::Vector<11,float> Vector11f;
typedef TooN::Matrix<3,3,float> Matrix3f;
typedef TooN::Matrix<6,6,float> Matrix6f;
typedef TooN::Matrix<7,7,float> Matrix7f;
typedef TooN::Matrix<11,11,float> Matrix11f;

inline Matrix3f invert(const rebvio::types::Matrix3f& _in) {
	Matrix3f out;
  out(0,0)=  _in(2,2)*_in(1,1)-_in(2,1)*_in(1,2); out(0,1)=-(_in(2,2)*_in(0,1)-_in(2,1)*_in(0,2));out(0,2)=  _in(1,2)*_in(0,1)-_in(1,1)*_in(0,2);
  out(1,0)=-(_in(2,2)*_in(1,0)-_in(2,0)*_in(1,2));out(1,1)=  _in(2,2)*_in(0,0)-_in(2,0)*_in(0,2); out(1,2)=-(_in(1,2)*_in(0,0)-_in(1,0)*_in(0,2));
  out(2,0)=  _in(2,1)*_in(1,0)-_in(2,0)*_in(1,1); out(2,1)=-(_in(2,1)*_in(0,0)-_in(2,0)*_in(0,1));out(2,2)=  _in(1,1)*_in(0,0)-_in(1,0)*_in(0,1);
  return out/TooN::determinant(_in);
}

} /* namespace types */
} /* namespace rebvio */



#endif /* INCLUDE_REBVIO_TYPES_PRIMITIVES_HPP_ */
