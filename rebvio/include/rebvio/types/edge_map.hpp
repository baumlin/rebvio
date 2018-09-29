/*
 * edge_map.hpp
 *
 *  Created on: Sep 2, 2018
 *      Author: baumlin
 */

#ifndef INCLUDE_REBVIO_TYPES_EDGE_MAP_HPP_
#define INCLUDE_REBVIO_TYPES_EDGE_MAP_HPP_

#include <memory>
#include <vector>
#include <unordered_map>
#include "rebvio/types/keyline.hpp"
#include "rebvio/types/imu.hpp"

namespace rebvio {
namespace types {


class EdgeMap {

public:
	using SharedPtr = std::shared_ptr<rebvio::types::EdgeMap>;

public:
	EdgeMap() = delete;
	EdgeMap(rebvio::Camera::SharedPtr _camera, int _size, uint64_t _ts);
	rebvio::types::KeyLine& operator[](int _idx);
	int size();
	std::vector<rebvio::types::KeyLine>& keylines();
	uint64_t ts();
	types::Float& threshold();
	rebvio::types::IntegratedImu& imu();
	unsigned int& matches();
	std::unordered_map<unsigned int,unsigned int>& mask();

	types::Float estimateQuantile(types::Float _sigma_rho_min, types::Float _sigma_rho_max, types::Float _percentile, int _num_bins);
	void rotateKeylines(const rebvio::types::Matrix3f& _R);

	int forwardMatch(rebvio::types::EdgeMap::SharedPtr _map);

	int searchMatch(const rebvio::types::KeyLine& _keyline, const rebvio::types::Vector3f& _vel, const rebvio::types::Matrix3f& _Rvel,
									const rebvio::types::Matrix3f& _Rback, types::Float _min_thr_mod, types::Float _min_thr_ang, types::Float _max_radius, types::Float _loc_uncertainty);

	int directedMatch(rebvio::types::EdgeMap::SharedPtr _map, const rebvio::types::Vector3f& _vel, const rebvio::types::Matrix3f& _Rvel, const rebvio::types::Matrix3f& _Rback,
										int& _kf_matches, types::Float _min_thr_mod, types::Float _min_thr_ang, types::Float _max_radius, types::Float _loc_uncertainty);

	int regularize1Iter(types::Float _threshold);

private:
	inline int getIndex(types::Float _row, types::Float _col) {
		int row = std::round(_row);
		int col = std::round(_col);
		if(row >= camera_->rows_ || row < 0 || col >= camera_->cols_ || col < 0) return -1;
		return row*camera_->cols_+col;
	}

private:
	uint64_t ts_;
	std::vector<rebvio::types::KeyLine> keylines_;
	std::unordered_map<unsigned int,unsigned int> keylines_mask_; //!< A lookup table with the image index as key and keyline index in keylines_ as value
	types::Float threshold_; //!< Threshold for keyline detection
	rebvio::types::IntegratedImu imu_;
	rebvio::Camera::SharedPtr camera_;
	unsigned int matches_;
};

} /* namespace types */
} /* namespace rebvio */

#endif /* INCLUDE_REBVIO_TYPES_EDGE_MAP_HPP_ */
