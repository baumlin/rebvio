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

struct DistanceFieldElement {
	int id = {-1};
	int distance = {std::numeric_limits<int>::max()};
};

class DistanceField {
public:
	DistanceField(int _rows, int _cols, float _search_range) :
		rows_(_rows),
		cols_(_cols),
		search_range_(_search_range)
	{
		field_ = new DistanceFieldElement[rows_*cols_];
	}

	~DistanceField() {
		delete[] field_;
	}

	/**
	 * Reset distance field, i.e. set all id's = -1 (and leave other quantities) and (re-)build
	 */
	void build(rebvio::types::EdgeMap::SharedPtr _map) {

		map_ = _map;
		int size = rows_*cols_;
		for(int idx = 0; idx < size; ++idx) field_[idx].id = -1;
		for(int idx = 0; idx < _map->size(); ++idx) {

			const rebvio::types::KeyLine& keyline = (*_map)[idx];
			// threshold on keyline gradient norm
			if(_map->threshold() > 0.0 && keyline.gradient_norm < _map->threshold()) continue;
			// get index of pixels in +/- direction of keyline gradient
			for(int r = -search_range_; r < search_range_; ++r) {
				int field_idx = getIndex((keyline.gradient[1]/keyline.gradient_norm)*float(r)+keyline.pos[1],
																 (keyline.gradient[0]/keyline.gradient_norm)*float(r)+keyline.pos[0]);
				if(field_idx < 0) continue;
				DistanceFieldElement& element = field_[field_idx];
				if(element.id >= 0 && element.distance < std::abs(r)) continue;
				element.distance = std::abs(r);
				element.id = idx;
			}
		}
	}

	DistanceFieldElement& operator[] (int _index) { return field_[_index]; }

	rebvio::types::EdgeMap::SharedPtr map() { return map_; }

private:
	inline int getIndex(float _row, float _col) {
		int row = std::round(_row);
		int col = std::round(_col);
		if(row >= rows_ || row < 0 || col >= cols_ || col < 0) return -1;
		return row*cols_+col;
	}

private:
	DistanceFieldElement* field_;
	uint rows_;
	uint cols_;
	float search_range_;
	rebvio::types::EdgeMap::SharedPtr map_;
};

class KaGMEKBias {
public:
	struct Config {
		rebvio::types::Vector3f a_v;
		rebvio::types::Vector3f a_s;
		float G;
		rebvio::types::Vector7f x_p;
		rebvio::types::Matrix3f Rv;
		rebvio::types::Matrix3f Rs;
		float Rg;
		rebvio::types::Matrix7f Pp;

		Config(const rebvio::types::Vector3f& _a_v, const rebvio::types::Vector3f& _a_s, float _G, const rebvio::types::Vector7f& _x_p,
					 const rebvio::types::Matrix3f& _Rv, const rebvio::types::Matrix3f& _Rs, float _Rg, const rebvio::types::Matrix7f& _Pp) :
						 a_v(_a_v), a_s(_a_s), G(_G), x_p(_x_p), Rv(_Rv), Rs(_Rs), Rg(_Rg), Pp(_Pp) {}
	};

public:
	KaGMEKBias(KaGMEKBias::Config& _config);
	~KaGMEKBias();
	bool problem(rebvio::types::Matrix7f& _JtJ, rebvio::types::Vector7f& _JtF, const rebvio::types::Vector7f& _X);
	int gaussNewton(rebvio::types::Vector7f& _X, int _iter_max, float _a_tol = 0.0, float _r_tol = 0.0);

private:
	inline float saturate(float _t, float _limit);

private:
	KaGMEKBias::Config config_;

};

class EdgeTracker {
public:
	struct Config {
		float search_range{40.0};				//!< Pixel search range for tracking and mapping
		float reweight_distance{2.0}; 	//!< Reweigh error residual in Huber Loss Function
		float match_treshold{0.5}; 			//!< Threshold on the keyline gradient dot product
		float match_threshold_module{1.0};
		float match_threshold_angle{45.0};
		unsigned int min_match_threshold{0}; 	//!< Minimum number of consecutive matches for a keyline
		unsigned int iterations{5}; 						//!< Number of iterations for tracker
		unsigned int global_min_matches_threshold{500}; //!< Minimum number of keyline matches for tracking and mapping
		float pixel_uncertainty{1};	//!< Pixel uncertainty for the correction step
		float quantile_cutoff{0.9}; //!< Percentile of the keylines to use
		int quantile_num_bins{100}; //!< Number of bins in the histogram for percentile calculation
	};

public:
	EdgeTracker(rebvio::Camera::SharedPtr);
	~EdgeTracker();

	EdgeTracker::Config& config();

	rebvio::types::EdgeMap::SharedPtr detect(rebvio::types::Image&,int);
	cv::Mat& getMask();
	void buildDistanceField(rebvio::types::EdgeMap::SharedPtr _map);


	static inline bool testfk(const rebvio::types::KeyLine& _keyline1, const rebvio::types::KeyLine& _keyline2, const float& _simil_t);

	/**
	 * Method to search for keyline match in distance field and calculate gradients
	 * \param _f_inx Distance field index where to look
	 * \param _df_dx Gradient vector x component
	 * \param _df_dy Gradient vector y component
	 * \param keyline Keyline
	 * \param _px Keyline x position with origin in principal point
	 * \param _py Keyline y position with origin in principal point
	 * \param _max_r Maximum allowed radius
	 * \param _simil_t Match threshold
	 * \param _mnum Match counter
	 * \param _fi Pixel residual
	 */
	float calculatefJ(rebvio::types::EdgeMap::SharedPtr _map, int _f_inx, float& _df_dx, float& _df_dy, rebvio::types::KeyLine& _keyline,
			const float& _px, const float& _py, const float& _max_r, const float& _simil_t, int& _mnum, float& _fi);

	float tryVel(rebvio::types::EdgeMap::SharedPtr _map, rebvio::types::Matrix3f& _JtJ, rebvio::types::Vector3f& _JtF, const rebvio::types::Vector3f& _vel,
			 float _sigma_rho_min, float* _residuals);


	float minimizeVel(rebvio::types::EdgeMap::SharedPtr _map, rebvio::types::Vector3f& _vel, rebvio::types::Matrix3f& _Rvel);

	bool extRotVel(rebvio::types::EdgeMap::SharedPtr _map, const rebvio::types::Vector3f& _vel, rebvio::types::Matrix6f& _Wx, rebvio::types::Matrix6f& _Rx, rebvio::types::Vector6f& _X);

	void correctBias(rebvio::types::Vector6f& _X, rebvio::types::Matrix6f& _Wx, rebvio::types::Vector3f& _Gb,
									 rebvio::types::Matrix3f& _Wb, const rebvio::types::Matrix3f& _Rg, const rebvio::types::Matrix3f& _Rb);

	void estimateLs4Acceleration(const rebvio::types::Vector3f& _vel, rebvio::types::Vector3f& _acc,
															 const rebvio::types::Matrix3f& _R, float _dt);

	void estimateMeanAcceleration(const rebvio::types::Vector3f _sacc, rebvio::types::Vector3f& _acc, const rebvio::types::Matrix3f& _R);

	float estimateBias(const rebvio::types::Vector3f& _sacc, const rebvio::types::Vector3f& _facc, float _kP, const rebvio::types::Matrix3f _Rot,
											rebvio::types::Vector7f& _X, rebvio::types::Matrix7f& _P, const rebvio::types::Matrix3f& _Qg, const rebvio::types::Matrix3f& _Qrot,
	                    const rebvio::types::Matrix3f& _Qbias, float _QKp, float _Rg, const rebvio::types::Matrix3f& _Rs,
											const rebvio::types::Matrix3f& _Rf, rebvio::types::Vector3f& _g_est, rebvio::types::Vector3f& _b_est, const rebvio::types::Matrix6f& _Wvw,
											rebvio::types::Vector6f& _Xvw, float _g_gravit);


private:
	EdgeTracker::Config config_;
	rebvio::Camera::SharedPtr camera_;
	rebvio::EdgeDetector detector_;
	rebvio::DistanceField distance_field_;
	unsigned int frame_count_;				//!< Number of frame minimizations performed


};

} /* namespace rebvio */

#endif /* INCLUDE_REBVIO_EDGE_TRACKER_HPP_ */
