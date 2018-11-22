/*
 * keyline.hpp
 *
 *  Created on: Aug 29, 2018
 *      Author: baumlin
 */

#ifndef INCLUDE_REBVIO_TYPES_KEYLINE_HPP_
#define INCLUDE_REBVIO_TYPES_KEYLINE_HPP_

#include <rebvio/types/definitions.hpp>
#include <cmath>

namespace rebvio {
namespace types {

constexpr types::Float RHO_MAX=20.0;
constexpr types::Float RHO_MIN=1e-3;
constexpr types::Float RHO_INIT=1.0;

/**
 * \brief Struct holding a keyline primitive
 */
struct KeyLine {
	Point2Df pos;                       //!< Subpixel keyline position in pixel coordinates (origin in upper left corner)
	Point2Df pos_img;                   //!< Keyline position in image coordinates (origin at principal point)
	Point2Df match_pos_img;             //!< Position of matched keyline in image coordinates
	Vector2f gradient;                  //!< Keyline gradient
	Vector2f match_gradient;            //!< Gradient of matched keyline
	types::Float gradient_norm;         //!< Norm of keyline gradient
	types::Float match_gradient_norm;   //!< Norm of matched keyline gradient
	types::Float rho;                   //!< Inverse depth of keyline
	types::Float sigma_rho;             //!< Inverse depth uncertainty of keyline
	int id;                             //!< ID of the keyline
	int id_prev;                        //!< ID of the previous consecutive keyline
	int id_next;                        //!< ID of the next consecutive keyline
	int match_id;                       //!< ID of the matching keyline
	int match_id_forward;               //!< ID of the matching keyline by forward matching
	int match_id_keyframe;              //!< ID of the matching keyline in the last keyframe
	unsigned int matches;               //!< Number of consecutive matches
	KeyLine() = delete;
	KeyLine(Point2Df& _pos, Vector2f& _gradient, Point2Df&& _pos_img) :
		pos(_pos),
		pos_img(_pos_img),
		match_pos_img(_pos_img),
		gradient(_gradient),
		match_gradient(TooN::Zeros),
		gradient_norm(std::sqrt(gradient[0]*gradient[0]+gradient[1]*gradient[1])),
    match_gradient_norm(0.0),
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
