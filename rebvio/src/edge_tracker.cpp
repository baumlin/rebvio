/*
 * edge_tracker.cpp
 *
 *  Created on: Sep 2, 2018
 *      Author: baumlin
 */

#include <rebvio/edge_tracker.hpp>

namespace rebvio {

EdgeTracker::EdgeTracker() {
	// TODO Auto-generated constructor stub

}

EdgeTracker::~EdgeTracker() {
	// TODO Auto-generated destructor stub
}

rebvio::types::EdgeMapPtr EdgeTracker::detect(rebvio::types::Image& _image,int _num_bins) {
	return detector_.detect(_image,_num_bins);
}

cv::Mat& EdgeTracker::getMask() {
	return detector_.getMask();
}

} /* namespace rebvio */
