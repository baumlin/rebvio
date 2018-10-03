/*
 * edge_map.cpp
 *
 *  Created on: Sep 2, 2018
 *      Author: baumlin
 */

#include "rebvio/types/edge_map.hpp"
#include "rebvio/util/timer.hpp"

namespace rebvio {
namespace types {

EdgeMap::EdgeMap(std::shared_ptr<rebvio::Camera> _camera, int _size, uint64_t _ts_us, rebvio::types::EdgeMapConfig::SharedPtr _config) :
		config_(_config),
		camera_(_camera),
		ts_us_(_ts_us),
		threshold_(-1.0),
		matches_(0)
{
	keylines_.reserve(_size);
}

rebvio::types::KeyLine& EdgeMap::operator[](int _idx) { return keylines_[_idx]; }

int EdgeMap::size() { return keylines_.size(); }

std::vector<rebvio::types::KeyLine>& EdgeMap::keylines() { return keylines_; }

uint64_t EdgeMap::ts_us() { return ts_us_; }

const types::Float& EdgeMap::threshold() const { return threshold_; }

void EdgeMap::threshold(const types::Float& _treshold) { threshold_ = _treshold; }

rebvio::types::IntegratedImu& EdgeMap::imu() { return imu_; }

std::unordered_map<unsigned int,unsigned int>& EdgeMap::mask() { return keylines_mask_; }

types::Float EdgeMap::estimateQuantile(types::Float _percentile, int _num_bins) {
	int histogram[_num_bins] = {0};
	for(int idx = 0; idx < size(); ++idx) {
		int i = _num_bins*(keylines_[idx].sigma_rho-types::RHO_MIN)/(types::RHO_MAX-types::RHO_MIN);
		i = (i > _num_bins-1) ? (_num_bins-1) : i;
		i = (i < 0) ? 0 : i;
		++histogram[i];
	}
	types::Float sigma_rho = 1e3;
	for(int i = 0, a = 0; i < _num_bins; ++i) {
		if(a > _percentile*size()) {
			sigma_rho = types::Float(i)*(types::RHO_MAX-types::RHO_MIN)/types::Float(_num_bins)+types::RHO_MIN;
			break;
		}
		a+=histogram[i];
	}
	return sigma_rho;
}

void EdgeMap::rotateKeylines(const rebvio::types::Matrix3f& _R){
	REBVIO_TIMER_TICK();
	for(rebvio::types::KeyLine& keyline : keylines_) {
		types::Vector3f q = _R*TooN::makeVector(keyline.pos_img[0]/camera_->fm_,keyline.pos_img[1]/camera_->fm_,1.0);
		if(fabs(q[2]) > 0.0) {
			keyline.pos_img[0] = q[0]/q[2]*camera_->fm_;
			keyline.pos_img[1] = q[1]/q[2]*camera_->fm_;
			keyline.rho /= q[2];
			keyline.sigma_rho /= q[2];
		}
		q = _R*TooN::makeVector(keyline.gradient[0],keyline.gradient[1],0.0);
		keyline.gradient[0] = q[0];
		keyline.gradient[1] = q[1];
	}
	REBVIO_TIMER_TOCK();
}

int EdgeMap::forwardMatch(std::shared_ptr<rebvio::types::EdgeMap> _map) {
	REBVIO_TIMER_TICK();
	unsigned int num_matches = 0;
//	char count[_map->size()] = {0};
	int nm = 0;

	for(int idx = 0; idx < size(); ++idx) {
		const types::KeyLine& keyline = keylines_[idx];
		const int& idx_f = keyline.match_id_forward;	// Forward match might be available, from tryVelRot()
		if(idx_f < 0) continue;
		rebvio::types::KeyLine& map_keyline = (*_map)[idx_f];
		if(map_keyline.match_id >= 0 && map_keyline.rho > keyline.rho) continue;

//		if(++count[idx_f] > 1) --num_matches; //TODO: should double matches count?
		map_keyline.rho = keyline.rho;
		map_keyline.sigma_rho = keyline.sigma_rho;
		map_keyline.matches = keyline.matches+1;
		map_keyline.match_id = idx;
		map_keyline.match_pos_img = keyline.pos_img;
		map_keyline.match_gradient = keyline.gradient;
		map_keyline.match_gradient_norm = keyline.gradient_norm;
		map_keyline.match_id_keyframe = keyline.match_id_keyframe;

		++num_matches;
	}
	_map->matches_ = num_matches;
	REBVIO_TIMER_TOCK();
	return num_matches;
}

int EdgeMap::searchMatch(const rebvio::types::KeyLine& _keyline, const rebvio::types::Vector3f& _vel, const rebvio::types::Matrix3f& _Rvel,
								const rebvio::types::Matrix3f& _Rback, types::Float _max_radius) {

	const types::Float cang_min_edge = std::cos(config_->match_threshold_angle*M_PI/180.0);

	types::Vector3f p_m3 = _Rback*TooN::makeVector(_keyline.pos_img[0],_keyline.pos_img[1],camera_->fm_);
	types::Float pmx = p_m3[0]*camera_->fm_/p_m3[2];
	types::Float pmy = p_m3[1]*camera_->fm_/p_m3[2];
	types::Float k_rho = _keyline.rho*camera_->fm_/p_m3[2];

	types::Float pi0x = pmx+camera_->cx_;
	types::Float pi0y = pmy+camera_->cy_;

	types::Float t_x = -(_vel[0]*camera_->fm_-_vel[2]*pmx);
	types::Float t_y = -(_vel[1]*camera_->fm_-_vel[2]*pmy);
	types::Float norm_t = std::sqrt(t_x*t_x+t_y*t_y);

	types::Vector3f DrDv = TooN::makeVector(camera_->fm_,camera_->fm_,-(pmx+pmy));
	types::Float sigma2_t = (DrDv.as_row()*_Rvel*DrDv.as_col())(0,0);


	types::Float dq_min = 0.0;
	types::Float dq_max = 0.0;
	types::Float dq_rho = 0.0;
	int t_steps = 0;
	if(norm_t > 1e-6) {
		t_x /= norm_t;
		t_y /= norm_t;
		dq_rho = norm_t*k_rho;
		dq_min = std::max(types::Float(0.0),norm_t*(k_rho-_keyline.sigma_rho))-config_->pixel_uncertainty_match;
		dq_max = std::min(_max_radius,norm_t*(k_rho+_keyline.sigma_rho))+config_->pixel_uncertainty_match;
		if(dq_rho > dq_max) {
			dq_rho = 0.5*(dq_max+dq_min);
			t_steps = static_cast<int>(dq_rho+0.5);
		} else {
			t_steps = static_cast<int>(std::max(dq_max-dq_rho,dq_rho-dq_min));
		}
	} else {
		t_x = _keyline.gradient[0];
		t_y = _keyline.gradient[1];
		norm_t = _keyline.gradient_norm;
		t_x /= norm_t;
		t_y /= norm_t;
		norm_t = 1.0;
		dq_min = -_max_radius-config_->pixel_uncertainty_match;
		dq_max = _max_radius+config_->pixel_uncertainty_match;
		dq_rho = 0.0;
		t_steps = dq_max;
	}

	types::Float tn = dq_rho;
	types::Float tp = dq_rho+1;
	for(int t_i = 0; t_i < t_steps; ++t_i, ++tp, --tn) {
		for(int i_idx = 0; i_idx < 2; ++i_idx) {
			types::Float t;
			if(i_idx) {
				t = tp;
				if(t > dq_max) continue;
			} else {
				t = tn;
				if(t < dq_min) continue;
			}

			int idx = getIndex(t_y*t+pi0y,t_x*t+pi0x);
			if(idx < 0) continue;

			auto search = keylines_mask_.find(idx);	// keyline_mask_ is an map with image index as key and index of according keyline in keylines_ as value
			if(search != keylines_mask_.end()) {
				const types::KeyLine& keyline = keylines_[search->second];

				types::Float cang = (keyline.gradient[0]*_keyline.gradient[0]+keyline.gradient[1]*_keyline.gradient[1])/(keyline.gradient_norm*_keyline.gradient_norm);
				if(cang < cang_min_edge || std::fabs(keyline.gradient_norm/_keyline.gradient_norm-1.0) > config_->match_threshold_norm) continue;

				types::Float v_rho_dr = (config_->pixel_uncertainty_match*config_->pixel_uncertainty_match+keyline.sigma_rho*keyline.sigma_rho*norm_t*norm_t+sigma2_t*keyline.rho*keyline.rho);
				if((t-norm_t*keyline.rho)*(t-norm_t*keyline.rho) > v_rho_dr) continue;

				return search->second;
			}

		}
	}

	return -1;
}

int EdgeMap::directedMatch(std::shared_ptr<rebvio::types::EdgeMap> _map, const rebvio::types::Vector3f& _vel, const rebvio::types::Matrix3f& _Rvel,
		                       const rebvio::types::Matrix3f& _Rback, int& _kf_matches, types::Float _max_radius) {

	REBVIO_TIMER_TICK();
	matches_ = 0;
	_kf_matches = 0;

	types::Vector3f vel = _Rback*_vel;
	types::Matrix3f Rvel = _Rback*_Rvel*_Rback.T();

	for(int idx = 0; idx < size(); ++idx) {

		types::KeyLine& keyline = keylines_[idx];
		int idx_match = _map->searchMatch(keyline,vel,Rvel,_Rback,_max_radius);
		if(idx_match < 0) continue;

		const types::KeyLine& matched_keyline = (*_map)[idx_match];
		keyline.rho = matched_keyline.rho;
		keyline.sigma_rho = matched_keyline.sigma_rho;
		keyline.match_id = idx_match;
		keyline.matches = matched_keyline.matches+1;
		keyline.match_pos_img = matched_keyline.pos_img;
		keyline.match_gradient = matched_keyline.gradient;
		keyline.match_gradient_norm = matched_keyline.gradient_norm;
		keyline.match_id_keyframe = matched_keyline.match_id_keyframe;

		if(keyline.match_id_keyframe >= 0) ++_kf_matches;
		++matches_;
	}

	REBVIO_TIMER_TOCK();
	return matches_;
}

int EdgeMap::regularize1Iter() {
	REBVIO_TIMER_TICK();
	int r_num = 0;
	types::Float r[size()];
	types::Float s[size()];
	bool set[size()];
	for(int idx = 0; idx < size(); ++idx) {
		set[idx] = false;

		types::KeyLine& keyline = keylines_[idx];
		if(keyline.id_next < 0 || keyline.id_prev < 0) continue;

		types::KeyLine& keyline_next = keylines_[keyline.id_next];
		types::KeyLine& keyline_prev = keylines_[keyline.id_prev];

		// First propabilistic test using inverse depth and its uncertainty
		if((keyline_next.rho-keyline_prev.rho)*(keyline_next.rho-keyline_prev.rho) > (keyline_next.sigma_rho*keyline_next.sigma_rho+keyline_prev.sigma_rho*keyline_prev.sigma_rho)) continue;

		// Second morphological test using gradients
		types::Float alpha = (keyline_next.gradient[0]*keyline_prev.gradient[0]+keyline_next.gradient[1]*keyline_prev.gradient[1])/(keyline_next.gradient_norm*keyline_prev.gradient_norm);
		if(alpha < config_->regularization_threshold) continue;

		alpha = (alpha-config_->regularization_threshold)/(1.0-config_->regularization_threshold);
		alpha /= std::fabs(keyline_next.rho-keyline_prev.rho)/(keyline_next.sigma_rho+keyline_prev.sigma_rho)+1.0;
		types::Float wr = 1.0/(keyline.sigma_rho*keyline.sigma_rho);
		types::Float wrn = alpha/(keyline_next.sigma_rho*keyline_next.sigma_rho);
		types::Float wrp = alpha/(keyline_prev.sigma_rho*keyline_prev.sigma_rho);

		r[idx] = (keyline.rho*wr+keyline_next.rho*wrn+keyline_prev.rho*wrp)/(wr+wrn+wrp);
		s[idx] = (keyline.sigma_rho*wr+keyline_next.sigma_rho*wrn+keyline_prev.sigma_rho*wrp)/(wr+wrn+wrp);
		set[idx] = true;
		++r_num;
	}
	for(int idx = 0; idx < size(); ++idx) {
		if(set[idx]) {
			keylines_[idx].rho = r[idx];
			keylines_[idx].sigma_rho = s[idx];
		}
	}
	REBVIO_TIMER_TOCK();
	return r_num;
}

} /* namespace types */
} /* namespace rebvio */
