/*
 * edge_tracker.h
 *
 *  Created on: Sep 2, 2018
 *      Author: baumlin
 */

#ifndef INCLUDE_REBVIO_EDGE_TRACKER_HPP_
#define INCLUDE_REBVIO_EDGE_TRACKER_HPP_

#include "rebvio/types/edge_map.hpp"
#include "rebvio/edge_detector.hpp"

namespace rebvio {

class EdgeTracker {
public:
	EdgeTracker();
	~EdgeTracker();

	rebvio::types::EdgeMapPtr detect(rebvio::types::Image&,int);
	cv::Mat& getMask();
	void buildDistanceField(int _radius, float _min_mod);


private:
	rebvio::EdgeDetector detector_;

};

} /* namespace rebvio */

#endif /* INCLUDE_REBVIO_EDGE_TRACKER_HPP_ */
