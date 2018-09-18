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
#include "rebvio/types/keyline.hpp"

namespace rebvio {
namespace types {


class EdgeMap {
public:
	EdgeMap(int _size, uint64_t _ts);
	rebvio::types::KeyLine& operator[](int _idx);
	int size();
	std::vector<rebvio::types::KeyLine>& keylines();
	uint64_t ts();

	float estimateQuantile(float _sigma_rho_min, float _sigma_rho_max, float _percentile, int _num_bins);

private:
	uint64_t ts_;
	std::vector<rebvio::types::KeyLine> keylines_;
};

typedef std::shared_ptr<rebvio::types::EdgeMap> EdgeMapPtr;

} /* namespace types */
} /* namespace rebvio */

#endif /* INCLUDE_REBVIO_TYPES_EDGE_MAP_HPP_ */
