/*
 * EdgeDetector.cpp
 *
 *  Created on: Aug 29, 2018
 *      Author: baumlin
 */

#include "rebvio/edge_detector.hpp"
#include "rebvio/types/edge_map.hpp"
#include "rebvio/util/timer.hpp"

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <memory>

namespace rebvio {

EdgeDetector::EdgeDetector(rebvio::CameraPtr _camera) :
	keylines_count_(0),
	keylines_size_(50000), //50000
	points_ref_(15000),
	points_max_(40000),
	points_tracked_(12000),
	plane_fit_size_(1),
	pos_neg_threshold_(0.4),
	dog_threshold_(0.095259868922420),
	threshold_(0.01),
	tuned_threshold_(threshold_),
	gain_(5e-7),
	max_threshold_(0.5),
	min_threshold_(0.005),
	max_image_value_(765), // (255*3)
	camera_(_camera),
	scale_space_(camera_)
{
	keylines_mask_ = cv::Mat(camera_->rows_,camera_->cols_,CV_32SC1,cv::Scalar(-1));
}

EdgeDetector::~EdgeDetector() {}

int EdgeDetector::getNumKeylines() const {
	return keylines_count_;
}

cv::Mat& EdgeDetector::getMask() {
	return keylines_mask_;
}

rebvio::types::EdgeMapPtr EdgeDetector::detect(rebvio::types::Image& _image,int _num_bins) {
	REBVIO_TIMER_TICK();
	rebvio::types::EdgeMapPtr map = std::make_shared<rebvio::types::EdgeMap>(keylines_size_,_image.ts);
	scale_space_.build(_image.data);

	if(gain_ > 0) {
		threshold_ -= gain_*float(points_ref_-keylines_count_);
		threshold_ = (threshold_ > max_threshold_) ? max_threshold_ : ((threshold_ < min_threshold_) ? min_threshold_ : threshold_);
	}
	buildMask(map);
	joinEdges(map);
	tuneThreshold(map,_num_bins);
	REBVIO_TIMER_TOCK();
	return map;
}

void EdgeDetector::buildMask(rebvio::types::EdgeMapPtr& _map) {
	REBVIO_TIMER_TICK();
	if(points_max_ > keylines_size_) points_max_ = keylines_size_;
	keylines_count_ = 0;

	static bool calc_phi = true;
	static Eigen::MatrixXf Pinv(3,(plane_fit_size_*2+1)*(plane_fit_size_*2+1));
	if(calc_phi) {
		Eigen::MatrixXf Phi((plane_fit_size_*2+1)*(plane_fit_size_*2+1),3);
		for(int row = -plane_fit_size_,k=0; row <= plane_fit_size_; ++row) {
			for(int col = -plane_fit_size_; col <= plane_fit_size_; ++col,++k) {
				Phi(k,0) = col;
				Phi(k,1) = row;
				Phi(k,2) = 1;
			}
		}
		Pinv = (Phi.transpose()*Phi).inverse()*Phi.transpose();
		calc_phi = false;
	}
	float pn_threshold = float((2.0*plane_fit_size_+1.0)*(2.0*plane_fit_size_+1.0))*pos_neg_threshold_;
	float gradient_threshold_squared = (threshold_*max_image_value_*dog_threshold_)*(threshold_*max_image_value_*dog_threshold_);
	float mag_threshold = (threshold_*max_image_value_)*(threshold_*max_image_value_);

	for(int row = plane_fit_size_; row < camera_->rows_-plane_fit_size_; ++row) {
		int* km_ptr = keylines_mask_.ptr<int>(row);
		const float* mag_ptr = scale_space_.gradient_mag_.ptr<float>(row);
		const float* dog0_ptr = scale_space_.dog_.ptr<float>(row-1);
		const float* dog1_ptr = scale_space_.dog_.ptr<float>(row);
		const float* dog2_ptr = scale_space_.dog_.ptr<float>(row+1);
		for(int col = plane_fit_size_; col < camera_->cols_-plane_fit_size_; ++col) {
			int idx = col+row*camera_->cols_;
			km_ptr[col] = -1;

			if(mag_ptr[col] < mag_threshold) continue;

			int pn = 0;
			Eigen::MatrixXf Y((plane_fit_size_*2+1)*(plane_fit_size_*2+1),1);
//			REBVIO_NAMED_TIMER_TICK(loop);
//
//			for(int r = -plane_fit_size_, k = 0; r <= plane_fit_size_; ++r) {
//				const float* dog_ptr = scale_space_.dog_.ptr<float>(row+r);
//				for(int c = -plane_fit_size_; c <= plane_fit_size_; ++c,++k) {
////					float dog = scale_space_.dog_.at<float>((row+r)*camera_->cols_+col+c);
//					float dog = dog_ptr[col+c];
//					Y(k,0) = dog;
//					pn = (dog > 0.0) ? pn+1 : pn-1;
//				}
//			}
//			REBVIO_NAMED_TIMER_TOCK(loop);
//			pn = 0;
//			REBVIO_NAMED_TIMER_TICK(pointer);
			Y(0,0) = dog0_ptr[col-1];
			pn = (Y(0,0) > 0.0) ? pn+1 : pn-1;
			Y(1,0) = dog0_ptr[col];
			pn = (Y(1,0) > 0.0) ? pn+1 : pn-1;
			Y(2,0) = dog0_ptr[col+1];
			pn = (Y(2,0) > 0.0) ? pn+1 : pn-1;
			Y(3,0) = dog1_ptr[col-1];
			pn = (Y(3,0) > 0.0) ? pn+1 : pn-1;
			Y(4,0) = dog1_ptr[col];
			pn = (Y(4,0) > 0.0) ? pn+1 : pn-1;
			Y(5,0) = dog1_ptr[col+1];
			pn = (Y(5,0) > 0.0) ? pn+1 : pn-1;
			Y(6,0) = dog2_ptr[col-1];
			pn = (Y(6,0) > 0.0) ? pn+1 : pn-1;
			Y(7,0) = dog2_ptr[col];
			pn = (Y(7,0) > 0.0) ? pn+1 : pn-1;
			Y(8,0) = dog2_ptr[col+1];
			pn = (Y(8,0) > 0.0) ? pn+1 : pn-1;
//			REBVIO_NAMED_TIMER_TOCK(pointer);


			if(fabs(pn) > pn_threshold) continue;

			Eigen::Vector3f theta = (Pinv*Y).col(0);
			float tmp = theta(2)/(theta(0)*theta(0)+theta(1)*theta(1));
			float xs = -theta(0)*tmp;
			float ys = -theta(1)*tmp;

			if(fabs(xs) > 0.5 || fabs(ys) > 0.5) continue;

			Eigen::Vector2f gradient(theta(0),theta(1)); // DoG gradient

			if(gradient.squaredNorm() < gradient_threshold_squared) continue;

			Eigen::Vector2f position(float(col)+xs,float(row)+ys);
			(*_map).keylines().emplace_back(types::KeyLine(idx,position,gradient));
			km_ptr[col] = keylines_count_;
			if(++keylines_count_ >= points_max_) { // now keylines_count_ == _map->size()
				int idx_boundary = camera_->rows_*camera_->cols_;
				for(++idx; idx < idx_boundary; ++idx) {
					keylines_mask_.at<int>(idx) = -1;
				}
				return;
			}
		}
	}
	REBVIO_TIMER_TOCK();
}

void EdgeDetector::joinEdges(rebvio::types::EdgeMapPtr& _map) {
	REBVIO_TIMER_TICK();
	for(int idx = 0; idx < keylines_count_; ++idx) {
		int x = int((*_map)[idx].pos(0)+0.5);
		int y = int((*_map)[idx].pos(1)+0.5);
		int id = nextPoint(_map,x,y,idx);
		if(id < 0) continue;
//		keylines_[id].id_prev = idx;
//		keylines_[idx].id_next = id;
		(*_map)[id].id_prev = idx;
		(*_map)[idx].id_next = id;
	}
	REBVIO_TIMER_TOCK();
}

int EdgeDetector::nextPoint(rebvio::types::EdgeMapPtr& _map, int _x, int _y, int _idx) {
//	REBVIO_TIMER_TICK();
	float tx = -(*_map)[_idx].gradient(1);
	float ty = (*_map)[_idx].gradient(0);
	int idx;
	if(ty>0.0) {
		if(tx>0.0) {
			if((idx = keylines_mask_.at<int>(_y,_x+1)) >= 0) { return idx; }
			if((idx = keylines_mask_.at<int>(_y+1,_x)) >= 0) { return idx; }
			if((idx = keylines_mask_.at<int>(_y+1,_x+1)) >= 0) { return idx; }
		} else {
			if((idx = keylines_mask_.at<int>(_y,_x-1)) >= 0) { return idx; }
			if((idx = keylines_mask_.at<int>(_y+1,_x)) >= 0) { return idx; }
			if((idx = keylines_mask_.at<int>(_y+1,_x-1)) >= 0) { return idx; }
		}
	} else {
		if(tx < 0.0) {
			if((idx = keylines_mask_.at<int>(_y,_x-1)) >= 0) { return idx; }
			if((idx = keylines_mask_.at<int>(_y-1,_x)) >= 0) { return idx; }
			if((idx = keylines_mask_.at<int>(_y-1,_x-1)) >= 0) { return idx; }
		} else {
			if((idx = keylines_mask_.at<int>(_y,_x+1)) >= 0) { return idx; }
			if((idx = keylines_mask_.at<int>(_y-1,_x)) >= 0) { return idx; }
			if((idx = keylines_mask_.at<int>(_y-1,_x+1)) >= 0) { return idx; }
		}
	}
//	REBVIO_TIMER_TOCK();
	return -1;

}

void EdgeDetector::tuneThreshold(rebvio::types::EdgeMapPtr _map, int _num_bins) {
//	REBVIO_TIMER_TICK();
	float max_dog = (*_map)[0].gradient_norm;
	float min_dog = max_dog;
	for(int idx = 1; idx < keylines_count_; ++idx) {
		if(max_dog < (*_map)[idx].gradient_norm) max_dog = (*_map)[idx].gradient_norm;
		if(min_dog > (*_map)[idx].gradient_norm) min_dog = (*_map)[idx].gradient_norm;
	}
	int histogram[_num_bins] = {0};
	for(int idx = 0; idx < keylines_count_; ++idx) {
		int i = _num_bins*(max_dog-(*_map)[idx].gradient_norm)/(max_dog-min_dog);
		i = (i > _num_bins-1) ? _num_bins-1 : i;
		i = (i < 0) ? 0 : i;
		++histogram[i];
	}
	int i = 0;
	for(int a = 0; i < _num_bins && a < points_tracked_; ++i, a+=histogram[i]);
	tuned_threshold_ = max_dog - float(i*(max_dog-min_dog))/float(_num_bins);
//	REBVIO_TIMER_TOCK();
}


} /* namespace rebvio */
