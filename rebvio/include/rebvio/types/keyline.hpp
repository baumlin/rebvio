/*
 * keyline.hpp
 *
 *  Created on: Aug 29, 2018
 *      Author: baumlin
 */

#ifndef INCLUDE_REBVIO_TYPES_KEYLINE_HPP_
#define INCLUDE_REBVIO_TYPES_KEYLINE_HPP_

#include <rebvio/types/primitives.hpp>
#include <eigen3/Eigen/Core>

namespace rebvio {
namespace types {

constexpr float RHO_MAX=20.0;
constexpr float RHO_MIN=1e-3;
constexpr float RHO_INIT=1.0;

struct KeyLine {
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	unsigned int idx;
	Eigen::Vector2f pos;
	Eigen::Vector2f gradient;
	float gradient_norm;
	float score;
	float rho;
	float sigma_rho;
	unsigned int id;
	unsigned int id_prev;
	unsigned int id_next;
	unsigned int matches;
	KeyLine() :
		idx(-1),
		pos(Eigen::Vector2f::Zero()),
		gradient(Eigen::Vector2f::Zero()),
		gradient_norm(0),
		score(0),
		rho(1.0),
		sigma_rho(20.0),
		id(-1),
		id_prev(-1),
		id_next(-1),
		matches(0)
	{}
	KeyLine(unsigned int _idx, Eigen::Vector2f& _pos, Eigen::Vector2f& _gradient) :
		idx(_idx),
		pos(_pos),
		gradient(_gradient),
		gradient_norm(gradient.norm()),
		score(0),
		rho(1),
		sigma_rho(20),
		id(-1),
		id_prev(-1),
		id_next(-1),
		matches(0)
	{}
};

} /* namespace types */
} /* namespace rebvio */



#endif /* INCLUDE_REBVIO_TYPES_KEYLINE_HPP_ */
