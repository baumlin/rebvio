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
	using SharedPtr =  std::shared_ptr<rebvio::types::EdgeMap>;

public:
	EdgeMap(rebvio::Camera::SharedPtr _camera, int _size, uint64_t _ts);
	rebvio::types::KeyLine& operator[](int _idx);
	int size();
	std::vector<rebvio::types::KeyLine>& keylines();
	uint64_t ts();
	float& threshold();
	rebvio::types::IntegratedImu& imu();
	unsigned int& matches();
	std::unordered_map<unsigned int,unsigned int>& mask();

	float estimateQuantile(float _sigma_rho_min, float _sigma_rho_max, float _percentile, int _num_bins);
	void rotateKeylines(const rebvio::types::Matrix3f& _R);

	int forwardMatch(rebvio::types::EdgeMap::SharedPtr _map);

	int searchMatch(const rebvio::types::KeyLine& _keyline, const rebvio::types::Vector3f& _vel, const rebvio::types::Matrix3f& _Rvel,
									const rebvio::types::Matrix3f& _Rback, float _min_thr_mod, float _min_thr_ang, float _max_radius, float _loc_uncertainty);

	int directedMatch(rebvio::types::EdgeMap::SharedPtr _map, const rebvio::types::Vector3f& _vel, const rebvio::types::Matrix3f& _Rvel, const rebvio::types::Matrix3f& _Rback,
										int& _kf_matches, float _min_thr_mod, float _min_thr_ang, float _max_radius, float _loc_uncertainty);

	inline int getIndex(float _row, float _col) {
		int row = std::round(_row);
		int col = std::round(_col);
		if(row >= camera_->rows_ || row < 0 || col >= camera_->cols_ || col < 0) return -1;
		return row*camera_->cols_+col;
	}

private:
	uint64_t ts_;
	std::vector<rebvio::types::KeyLine> keylines_;
	std::unordered_map<unsigned int,unsigned int> keylines_mask_; //!< A lookup table with the image index as key and keyline index in keylines_ as value
	float threshold_; //!< Threshold for keyline detection
	rebvio::types::IntegratedImu imu_;
	rebvio::Camera::SharedPtr camera_;
	unsigned int matches_;
};

typedef std::shared_ptr<rebvio::types::EdgeMap> EdgeMapPtr;

} /* namespace types */
} /* namespace rebvio */

#endif /* INCLUDE_REBVIO_TYPES_EDGE_MAP_HPP_ */
