/*
 * edge_tracker.cpp
 *
 *  Created on: Sep 2, 2018
 *      Author: baumlin
 */

#include <rebvio/edge_tracker.hpp>

namespace rebvio {

EdgeTracker::EdgeTracker(rebvio::CameraPtr _camera) :
	camera_(_camera),
	detector_(camera_)
{
	// TODO Auto-generated constructor stub
	distance_field_ = cv::Mat(camera_->rows_,camera_->cols_,CV_32SC1,cv::Scalar(-1));

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

void EdgeTracker::buildDistanceField(int _radius, float _min_mod) {

}

} /* namespace rebvio */
