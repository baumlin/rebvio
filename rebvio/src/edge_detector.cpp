/*
 * EdgeDetector.cpp
 *
 *  Created on: Aug 29, 2018
 *      Author: baumlin
 */

#include <rebvio/edge_map.hpp>
#include <rebvio/types/definitions.hpp>
#include "rebvio/edge_detector.hpp"
#include "rebvio/util/timer.hpp"
#include <iostream>
#include <memory>

namespace rebvio {

EdgeDetector::EdgeDetector(rebvio::Camera::SharedPtr _camera, rebvio::EdgeDetectorConfig::SharedPtr _config) :
	keylines_count_(0),
	config_(_config),
	auto_threshold_(config_->threshold),
	max_image_value_(765), // (255*3)
	camera_(_camera),
	scale_space_(camera_)
{
	keylines_mask_ = cv::Mat(camera_->rows_,camera_->cols_,CV_32SC1,cv::Scalar(-1));
}

EdgeDetector::~EdgeDetector() {}

rebvio::EdgeMap::SharedPtr EdgeDetector::detect(rebvio::types::Image& _image) {
	REBVIO_TIMER_TICK();

	if(config_->gain > 0) {
		config_->threshold -= config_->gain*types::Float(config_->keylines_ref-keylines_count_);
		config_->threshold = (config_->threshold > config_->max_threshold) ? config_->max_threshold : ((config_->threshold < config_->min_threshold) ? config_->min_threshold : config_->threshold);
	}
	rebvio::EdgeMap::SharedPtr map = buildEdgeMap(_image);
	joinEdges(map);
	tuneThreshold(map);

	REBVIO_TIMER_TOCK();
	return map;
}

rebvio::EdgeMap::SharedPtr EdgeDetector::buildEdgeMap(rebvio::types::Image& _image) {
	REBVIO_TIMER_TICK();

	// Build the scale space for edge detection
	scale_space_.build(_image.data);

	rebvio::EdgeMap::SharedPtr map = std::make_shared<rebvio::EdgeMap>(camera_,config_->keylines_max,_image.ts_us);

	// Reset quantities (keylines_mask_ is reset on the go in the loop below)
	keylines_count_ = 0;

	static bool calc_phi = true;
	static TooN::Matrix<TooN::Dynamic,TooN::Dynamic,types::Float> Pinv(3,(config_->plane_fit_size*2+1)*(config_->plane_fit_size*2+1));
	if(calc_phi) {
		TooN::Matrix<TooN::Dynamic,TooN::Dynamic,types::Float> Phi((config_->plane_fit_size*2+1)*(config_->plane_fit_size*2+1),3);
		for(int row = -config_->plane_fit_size,k=0; row <= config_->plane_fit_size; ++row) {
			for(int col = -config_->plane_fit_size; col <= config_->plane_fit_size; ++col,++k) {
				Phi(k,0) = col;
				Phi(k,1) = row;
				Phi(k,2) = 1;
			}
		}
		Pinv = types::invert(Phi.T()*Phi)*Phi.T();
		calc_phi = false;
	}
	types::Float pn_threshold = types::Float((2.0*config_->plane_fit_size+1.0)*(2.0*config_->plane_fit_size+1.0))*config_->pos_neg_threshold;
	types::Float gradient_threshold_squared = (config_->threshold*max_image_value_*config_->dog_threshold)*(config_->threshold*max_image_value_*config_->dog_threshold);
	types::Float mag_threshold = (config_->threshold*max_image_value_)*(config_->threshold*max_image_value_);

	for(int row = config_->plane_fit_size; row < camera_->rows_-config_->plane_fit_size; ++row) {
		int* km_ptr = keylines_mask_.ptr<int>(row);
		const types::Float* mag_ptr = scale_space_.mag().ptr<types::Float>(row);
		const types::Float* dog0_ptr = scale_space_.dog().ptr<types::Float>(row-1);
		const types::Float* dog1_ptr = scale_space_.dog().ptr<types::Float>(row);
		const types::Float* dog2_ptr = scale_space_.dog().ptr<types::Float>(row+1);
		for(int col = config_->plane_fit_size; col < camera_->cols_-config_->plane_fit_size; ++col) {
			int idx = col+row*camera_->cols_;
			km_ptr[col] = -1; // reset previous mask entry

			if(mag_ptr[col] < mag_threshold) continue;

			int pn = 0;
			TooN::Matrix<TooN::Dynamic,TooN::Dynamic,types::Float> Y((config_->plane_fit_size*2+1)*(config_->plane_fit_size*2+1),1);
			for(int r = -config_->plane_fit_size, k = 0; r <= config_->plane_fit_size; ++r) {
				const types::Float* dog_ptr = scale_space_.dog().ptr<types::Float>(row+r);
				for(int c = -config_->plane_fit_size; c <= config_->plane_fit_size; ++c,++k) {
					types::Float dog = dog_ptr[col+c];
					Y(k,0) = dog;
					pn = (dog > 0.0) ? pn+1 : pn-1;
				}
			}

			if(fabs(pn) > pn_threshold) continue;

			types::Vector3f theta = (Pinv*Y).T()[0];
			types::Float tmp = theta[2]/(theta[0]*theta[0]+theta[1]*theta[1]);
			types::Float xs = -theta[0]*tmp;
			types::Float ys = -theta[1]*tmp;

			if(fabs(xs) > 0.5 || fabs(ys) > 0.5) continue;

			types::Vector2f gradient = TooN::makeVector(theta[0],theta[1]); // DoG gradient

			if(gradient[0]*gradient[0]+gradient[1]*gradient[1] < gradient_threshold_squared) continue;

			types::Vector2f position = TooN::makeVector(types::Float(col)+xs,types::Float(row)+ys);
			map->keylines().emplace_back(types::KeyLine(position,gradient,TooN::makeVector(position[0]-camera_->cx_,position[1]-camera_->cy_)));
			km_ptr[col] = keylines_count_;
			map->mask().emplace(idx,keylines_count_);
			if(++keylines_count_ >= config_->keylines_max) { // now keylines_count_ == _map->size()
				int idx_boundary = camera_->rows_*camera_->cols_;
				for(++idx; idx < idx_boundary; ++idx) {
					keylines_mask_.at<int>(idx) = -1;
				}
				REBVIO_TIMER_TOCK();
				return map;
			}
		}
	}
	REBVIO_TIMER_TOCK();
	return map;
}

void EdgeDetector::joinEdges(rebvio::EdgeMap::SharedPtr& _map) {
	REBVIO_TIMER_TICK();
	for(int idx = 0; idx < _map->size(); ++idx) {
		types::KeyLine& keyline = (*_map)[idx];
		int x = static_cast<int>(keyline.pos[0]+0.5);
		int y = static_cast<int>(keyline.pos[1]+0.5);
		int id_next = nextKeylineIdx(_map,x,y,idx);
		if(id_next < 0) continue;

		(*_map)[id_next].id_prev = idx;
		keyline.id_next = id_next;
	}
	REBVIO_TIMER_TOCK();
}

int EdgeDetector::nextKeylineIdx(rebvio::EdgeMap::SharedPtr& _map, int _x, int _y, int _idx) {
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

void EdgeDetector::tuneThreshold(rebvio::EdgeMap::SharedPtr _map) {
//	REBVIO_TIMER_TICK();
	types::Float max_dog = (*_map)[0].gradient_norm;
	types::Float min_dog = max_dog;
	for(int idx = 1; idx < _map->size(); ++idx) {
		const types::KeyLine& keyline = (*_map)[idx];
		if(max_dog < keyline.gradient_norm) max_dog = keyline.gradient_norm;
		if(min_dog > keyline.gradient_norm) min_dog = keyline.gradient_norm;
	}
	int histogram[config_->num_bins] = {0};
	for(int idx = 0; idx < _map->size(); ++idx) {
		int i = config_->num_bins*(max_dog-_map->operator[](idx).gradient_norm)/(max_dog-min_dog);
		i = (i > config_->num_bins-1) ? config_->num_bins-1 : i;
		i = (i < 0) ? 0 : i;
		++histogram[i];
	}
	int i = 0;
	for(int a = 0; i < config_->num_bins && a < config_->keylines_max; i++, a+=histogram[i]);
	auto_threshold_ = max_dog - types::Float(i*(max_dog-min_dog))/types::Float(config_->num_bins);
	_map->threshold(auto_threshold_);
//	REBVIO_TIMER_TOCK();
}


} /* namespace rebvio */
