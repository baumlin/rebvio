/*
 * keyline.hpp
 *
 *  Created on: Aug 29, 2018
 *      Author: baumlin
 */

#ifndef INCLUDE_REBVIO_TYPES_KEYLINE_HPP_
#define INCLUDE_REBVIO_TYPES_KEYLINE_HPP_

#include "rebvio/types/primitives.hpp"
#include <TooN/TooN.h>
#include <cmath>

namespace rebvio {
namespace types {

constexpr types::Float RHO_MAX=20.0;
constexpr types::Float RHO_MIN=1e-3;
constexpr types::Float RHO_INIT=1.0;

struct KeyLine {
	unsigned int idx;
	Vector2f pos;
	Vector2f pos_hom;
	Vector2f match_pos_hom;
	Vector2f gradient;
	Vector2f match_gradient;
	types::Float gradient_norm;
	types::Float match_gradient_norm;
	types::Float score;
	types::Float rho;
	types::Float sigma_rho;
	int id;					//!< ID of the keyline
	int id_prev;			//!< ID of the previous consecutive keyline
	int id_next;			//!< ID of the next consecutive keyline
	int match_id; 					//!< ID of the matching keyline
	int match_id_forward; 	//!< ID of the matching keyline by forward matching
	int match_id_keyframe; //!< ID of the matching keyline in the last keyframe
	unsigned int matches;						//!< Number of consecutive matches
	KeyLine() :
		idx(-1),
		pos(TooN::Zeros),
		pos_hom(TooN::Zeros),
		match_pos_hom(TooN::Zeros),
		gradient(TooN::Zeros),
		match_gradient(TooN::Zeros),
		gradient_norm(0.0),
		match_gradient_norm(0.0),
		score(0.0),
		rho(1.0),
		sigma_rho(20.0),
		id(-1),
		id_prev(-1),
		id_next(-1),
		match_id(-1),
		match_id_forward(-1),
		match_id_keyframe(-1),
		matches(0)
	{}
	KeyLine(unsigned int _idx, Vector2f& _pos, Vector2f& _gradient, Vector2f&& _pos_hom) :
		idx(_idx),
		pos(_pos),
		pos_hom(_pos_hom),
		match_pos_hom(_pos_hom),
		gradient(_gradient),
		match_gradient(TooN::Zeros),
		gradient_norm(sqrt(gradient[0]*gradient[0]+gradient[1]*gradient[1])),
    match_gradient_norm(0.0),
		score(0),
		rho(1.0),
		sigma_rho(20.0),
		id(-1),
		id_prev(-1),
		id_next(-1),
		match_id(-1),
		match_id_forward(-1),
		match_id_keyframe(-1),
		matches(0)
	{}
};

} /* namespace types */
} /* namespace rebvio */



#endif /* INCLUDE_REBVIO_TYPES_KEYLINE_HPP_ */
