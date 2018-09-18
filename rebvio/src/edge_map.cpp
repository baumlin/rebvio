/*
 * edge_map.cpp
 *
 *  Created on: Sep 2, 2018
 *      Author: baumlin
 */

#include "rebvio/types/edge_map.hpp"

namespace rebvio {
namespace types {

EdgeMap::EdgeMap(int _size, uint64_t _ts) : ts_(_ts) {
	keylines_.reserve(_size);
}

rebvio::types::KeyLine& EdgeMap::operator[](int _idx) {
	return keylines_[_idx];
}

int EdgeMap::size() {
	return keylines_.size();
}

std::vector<rebvio::types::KeyLine>& EdgeMap::keylines() {
	return keylines_;
}

uint64_t EdgeMap::ts() {
	return ts_;
}

float EdgeMap::estimateQuantile(float _sigma_rho_min, float _sigma_rho_max, float _percentile, int _num_bins) {
	int histogram[_num_bins] = {0};
	for(int idx = 0; idx < keylines_.size(); ++idx) {
		int i = (keylines_[idx].sigma_rho-_sigma_rho_min)/(_sigma_rho_max-_sigma_rho_min);
		i = (i > _num_bins-1) ? (_num_bins-1) : i;
		i = (i < 0) ? 0 : i;
		++histogram[i];
	}
	float sigma_rho = 1e3;
	for(int i = 0, a = 0; i < _num_bins; ++i) {
		if(a > _percentile*keylines_.size()) {
			sigma_rho = float(i)*(_sigma_rho_max-_sigma_rho_min)/float(_num_bins)+_sigma_rho_min;
			break;
		}
		a+=histogram[i];
	}
	return sigma_rho;
}

} /* namespace types */
} /* namespace rebvio */
