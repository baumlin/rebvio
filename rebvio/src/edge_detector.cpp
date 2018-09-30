/*
 * EdgeDetector.cpp
 *
 *  Created on: Aug 29, 2018
 *      Author: baumlin
 */

#include "rebvio/edge_detector.hpp"
#include "rebvio/types/edge_map.hpp"
#include "rebvio/util/timer.hpp"
#include "rebvio/types/primitives.hpp"

#include <iostream>
#include <memory>

namespace rebvio {

EdgeDetector::EdgeDetector(rebvio::Camera::SharedPtr _camera) :
	keylines_count_(0),
	points_ref_(12000),
	points_max_(16000),
	plane_fit_size_(2),
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

rebvio::types::EdgeMap::SharedPtr EdgeDetector::detect(rebvio::types::Image& _image,int _num_bins) {
	REBVIO_TIMER_TICK();
	rebvio::types::EdgeMap::SharedPtr map = std::make_shared<rebvio::types::EdgeMap>(camera_,points_max_,_image.ts_us);
	scale_space_.build(_image.data);

	if(gain_ > 0) {
		threshold_ -= gain_*types::Float(points_ref_-keylines_count_);
		threshold_ = (threshold_ > max_threshold_) ? max_threshold_ : ((threshold_ < min_threshold_) ? min_threshold_ : threshold_);
	}
	buildMask(map);
	joinEdges(map);
	tuneThreshold(map,_num_bins);

	REBVIO_TIMER_TOCK();
	return map;
}

void EdgeDetector::buildMask(rebvio::types::EdgeMap::SharedPtr& _map) {
	REBVIO_TIMER_TICK();

	keylines_count_ = 0;

	static bool calc_phi = true;
	static TooN::Matrix<TooN::Dynamic,TooN::Dynamic,types::Float> Pinv(3,(plane_fit_size_*2+1)*(plane_fit_size_*2+1));
	if(calc_phi) {
		TooN::Matrix<TooN::Dynamic,TooN::Dynamic,types::Float> Phi((plane_fit_size_*2+1)*(plane_fit_size_*2+1),3);
		for(int row = -plane_fit_size_,k=0; row <= plane_fit_size_; ++row) {
			for(int col = -plane_fit_size_; col <= plane_fit_size_; ++col,++k) {
				Phi(k,0) = col;
				Phi(k,1) = row;
				Phi(k,2) = 1;
			}
		}
		Pinv = types::invert(Phi.T()*Phi)*Phi.T();
		calc_phi = false;
	}
	types::Float pn_threshold = types::Float((2.0*plane_fit_size_+1.0)*(2.0*plane_fit_size_+1.0))*pos_neg_threshold_;
	types::Float gradient_threshold_squared = (threshold_*max_image_value_*dog_threshold_)*(threshold_*max_image_value_*dog_threshold_);
	types::Float mag_threshold = (threshold_*max_image_value_)*(threshold_*max_image_value_);

	for(int row = plane_fit_size_; row < camera_->rows_-plane_fit_size_; ++row) {
		int* km_ptr = keylines_mask_.ptr<int>(row);
		const types::Float* mag_ptr = scale_space_.mag().ptr<types::Float>(row);
		const types::Float* dog0_ptr = scale_space_.dog().ptr<types::Float>(row-1);
		const types::Float* dog1_ptr = scale_space_.dog().ptr<types::Float>(row);
		const types::Float* dog2_ptr = scale_space_.dog().ptr<types::Float>(row+1);
		for(int col = plane_fit_size_; col < camera_->cols_-plane_fit_size_; ++col) {
			int idx = col+row*camera_->cols_;
			km_ptr[col] = -1;

			if(mag_ptr[col] < mag_threshold) continue;

			int pn = 0;
			TooN::Matrix<TooN::Dynamic,TooN::Dynamic,types::Float> Y((plane_fit_size_*2+1)*(plane_fit_size_*2+1),1);
			for(int r = -plane_fit_size_, k = 0; r <= plane_fit_size_; ++r) {
				const types::Float* dog_ptr = scale_space_.dog().ptr<types::Float>(row+r);
				for(int c = -plane_fit_size_; c <= plane_fit_size_; ++c,++k) {
					types::Float dog = dog_ptr[col+c];
					Y(k,0) = dog;
					pn = (dog > 0.0) ? pn+1 : pn-1;
				}
			}
//			REBVIO_NAMED_TIMER_TOCK(loop);  DONT USE THIS!
//			pn = 0;
//			REBVIO_NAMED_TIMER_TICK(pointer);
//			Y(0,0) = dog0_ptr[col-1];
//			pn = (Y(0,0) > 0.0) ? pn+1 : pn-1;
//			Y(1,0) = dog0_ptr[col];
//			pn = (Y(1,0) > 0.0) ? pn+1 : pn-1;
//			Y(2,0) = dog0_ptr[col+1];
//			pn = (Y(2,0) > 0.0) ? pn+1 : pn-1;
//			Y(3,0) = dog1_ptr[col-1];
//			pn = (Y(3,0) > 0.0) ? pn+1 : pn-1;
//			Y(4,0) = dog1_ptr[col];
//			pn = (Y(4,0) > 0.0) ? pn+1 : pn-1;
//			Y(5,0) = dog1_ptr[col+1];
//			pn = (Y(5,0) > 0.0) ? pn+1 : pn-1;
//			Y(6,0) = dog2_ptr[col-1];
//			pn = (Y(6,0) > 0.0) ? pn+1 : pn-1;
//			Y(7,0) = dog2_ptr[col];
//			pn = (Y(7,0) > 0.0) ? pn+1 : pn-1;
//			Y(8,0) = dog2_ptr[col+1];
//			pn = (Y(8,0) > 0.0) ? pn+1 : pn-1;
//			REBVIO_NAMED_TIMER_TOCK(pointer);


			if(fabs(pn) > pn_threshold) continue;

			types::Vector3f theta = (Pinv*Y).T()[0];
			types::Float tmp = theta[2]/(theta[0]*theta[0]+theta[1]*theta[1]);
			types::Float xs = -theta[0]*tmp;
			types::Float ys = -theta[1]*tmp;

			if(fabs(xs) > 0.5 || fabs(ys) > 0.5) continue;

			types::Vector2f gradient = TooN::makeVector(theta[0],theta[1]); // DoG gradient

			if(gradient[0]*gradient[0]+gradient[1]*gradient[1] < gradient_threshold_squared) continue;

			types::Vector2f position = TooN::makeVector(types::Float(col)+xs,types::Float(row)+ys);
			_map->keylines().emplace_back(types::KeyLine(position,gradient,TooN::makeVector(position[0]-camera_->cx_,position[1]-camera_->cy_)));
			km_ptr[col] = keylines_count_;
			_map->mask().emplace(idx,keylines_count_);
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

void EdgeDetector::joinEdges(rebvio::types::EdgeMap::SharedPtr& _map) {
	REBVIO_TIMER_TICK();
	for(int idx = 0; idx < _map->size(); ++idx) {
		int x = int((*_map)[idx].pos[0]+0.5);
		int y = int((*_map)[idx].pos[1]+0.5);
		int id = nextPoint(_map,x,y,idx);
		if(id < 0) continue;

		(*_map)[id].id_prev = idx;
		(*_map)[idx].id_next = id;
	}
	REBVIO_TIMER_TOCK();
}

int EdgeDetector::nextPoint(rebvio::types::EdgeMap::SharedPtr& _map, int _x, int _y, int _idx) {
//	REBVIO_TIMER_TICK();
	types::Float tx = -(*_map)[_idx].gradient[1];
	types::Float ty = (*_map)[_idx].gradient[0];
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

void EdgeDetector::tuneThreshold(rebvio::types::EdgeMap::SharedPtr _map, int _num_bins) {
//	REBVIO_TIMER_TICK();
	types::Float max_dog = (*_map)[0].gradient_norm;
	types::Float min_dog = max_dog;
	for(int idx = 1; idx < _map->size(); ++idx) {
		const types::KeyLine& keyline = (*_map)[idx];
		if(max_dog < keyline.gradient_norm) max_dog = keyline.gradient_norm;
		if(min_dog > keyline.gradient_norm) min_dog = keyline.gradient_norm;
	}
	int histogram[_num_bins] = {0};
	for(int idx = 0; idx < _map->size(); ++idx) {
		int i = _num_bins*(max_dog-(*_map)[idx].gradient_norm)/(max_dog-min_dog);
		i = (i > _num_bins-1) ? _num_bins-1 : i;
		i = (i < 0) ? 0 : i;
		++histogram[i];
	}
	int i = 0;
	for(int a = 0; i < _num_bins && a < points_max_; i++, a+=histogram[i]);
	tuned_threshold_ = max_dog - types::Float(i*(max_dog-min_dog))/types::Float(_num_bins);
	_map->threshold(tuned_threshold_);
//	REBVIO_TIMER_TOCK();
}


} /* namespace rebvio */
