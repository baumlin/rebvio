/*
 * edge_tracker.h
 *
 *  Created on: Sep 2, 2018
 *      Author: baumlin
 */

#ifndef INCLUDE_REBVIO_EDGE_TRACKER_HPP_
#define INCLUDE_REBVIO_EDGE_TRACKER_HPP_

#include "rebvio/types/edge_map.hpp"

namespace rebvio {

struct DistanceFieldElement {
	int id = {-1};
	int distance = {std::numeric_limits<int>::max()};
};

class DistanceField {
public:
	DistanceField(int _rows, int _cols, types::Float _search_range) :
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
		for(int i = 0; i < size; ++i) field_[i].id = -1;

		for(int idx = 0; idx < _map->size(); ++idx) {

			const rebvio::types::KeyLine& keyline = (*_map)[idx];
			// threshold on keyline gradient norm
			if(_map->threshold() > 0.0 && keyline.gradient_norm < _map->threshold()) continue;
			// get index of pixels in +/- direction of keyline gradient
			for(int r = -search_range_; r < search_range_; ++r) {
				int field_idx = getIndex((keyline.gradient[1]/keyline.gradient_norm)*types::Float(r)+keyline.pos[1],
																 (keyline.gradient[0]/keyline.gradient_norm)*types::Float(r)+keyline.pos[0]);
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
	inline int getIndex(types::Float _row, types::Float _col) {
		int row = std::round(_row);
		int col = std::round(_col);
		if(row >= rows_ || row < 0 || col >= cols_ || col < 0) return -1;
		return row*cols_+col;
	}

private:
	DistanceFieldElement* field_;
	unsigned int rows_;
	unsigned int cols_;
	types::Float search_range_;
	rebvio::types::EdgeMap::SharedPtr map_;
};

class KaGMEKBias {
public:
	struct Config {
		rebvio::types::Vector3f a_v;    //!< Visual acceleration
		rebvio::types::Vector3f a_s;    //!< Gravity-corrected acceleration
		types::Float G;                 //!< Standard gravity (norm of gravitational acceleration)
		rebvio::types::Vector7f x_p;    //!< Prior filter state
		rebvio::types::Matrix7f Pp;     //!< Prior filter state covariance
		rebvio::types::Matrix3f Rv;     //!< Observation noise of visual acceleration
		rebvio::types::Matrix3f Rs;     //!< Observation noise of gravity-corrected acceleration
		types::Float Rg;                //!< Observation noise of standard gravity

		Config(const rebvio::types::Vector3f& _a_v, const rebvio::types::Vector3f& _a_s, types::Float _G, const rebvio::types::Vector7f& _x_p,
					 const rebvio::types::Matrix3f& _Rv, const rebvio::types::Matrix3f& _Rs, types::Float _Rg, const rebvio::types::Matrix7f& _Pp) :
						 a_v(_a_v), a_s(_a_s), G(_G), x_p(_x_p), Rv(_Rv), Rs(_Rs), Rg(_Rg), Pp(_Pp) {}
		Config() = delete;
	};

public:
	KaGMEKBias(KaGMEKBias::Config& _config);
	KaGMEKBias() = delete;
	~KaGMEKBias();
	bool problem(rebvio::types::Matrix7f& _JtJ, rebvio::types::Vector7f& _JtF, const rebvio::types::Vector7f& _X);
	int gaussNewton(rebvio::types::Vector7f& _X, int _iter_max, types::Float _a_tol = 0.0, types::Float _r_tol = 0.0);

private:
	inline types::Float saturate(types::Float _t, types::Float _limit);

private:
	KaGMEKBias::Config config_;

};

class EdgeTracker {
public:
	struct Config {
		types::Float search_range{40.0};                 //!< Pixel search range for tracking and mapping
		types::Float reweight_distance{2.0};             //!< Reweigh error residual in Huber Loss Function
		types::Float match_treshold{0.5};                //!< Threshold on the keyline gradient dot product
		types::Float match_threshold_norm{1.0};          //!< Relative norm threshold between two keyline gradients for keyline matching
		types::Float match_threshold_angle{45.0};        //!< Angle threshold [deg] between two keyline gradients for keyline matching
		unsigned int min_match_threshold{0};             //!< Minimum number of consecutive matches for a keyline
		unsigned int iterations{5};                      //!< Number of iterations for tracker
		unsigned int global_min_matches_threshold{500};  //!< Minimum number of keyline matches for tracking and mapping
		types::Float pixel_uncertainty_match{2.0};       //!< Pixel uncertainty for the matching step
		types::Float pixel_uncertainty{1};               //!< Pixel uncertainty for the correction step
		types::Float quantile_cutoff{0.9};               //!< Percentile of the keylines to use
		int quantile_num_bins{100};                      //!< Number of bins in the histogram for percentile calculation
		types::Float regularization_threshold{0.5};      //!< Edgemap regularization threshold on angle beta between neighboring keyline gradients (threshold = cos(beta))
		types::Float reshape_q_abs{1e-4};                //!< EKF modeled absolute error on inverse depth
	};

public:
	EdgeTracker(rebvio::Camera::SharedPtr);
	EdgeTracker() = delete;
	~EdgeTracker();

	/**
	 * \brief Returns a reference to the configuration parameters
	 */
	EdgeTracker::Config& config();


	/**
	 * \brief Build the distance field of the edge map
	 * \param _map Edge map to build the distance field
	 */
	void buildDistanceField(rebvio::types::EdgeMap::SharedPtr _map);

	/**
	 * \brief Method to test the gradients of two keylines for similarity
	 * \param _keyline1 First keyline with gradient g1
	 * \param _keyline2 Second keyline with gradient g2
	 * \param _similarity_threshold Similarity threshold s.t. the test is passed if |g1*g2 - g2*g2|/(g2*g2) > threshold
	 */
	static inline bool testfk(const rebvio::types::KeyLine& _keyline1, const rebvio::types::KeyLine& _keyline2, const types::Float& _similarity_threshold);

	/**
	 * Method to search for keyline match in distance field and calculate gradients
	 * \param _f_inx Distance field index where to look
	 * \param _df_dx Gradient vector x component
	 * \param _df_dy Gradient vector y component
	 * \param keyline Keyline
	 * \param _px Keyline x position with origin in principal point
	 * \param _py Keyline y position with origin in principal point
	 * \param _mnum Match counter
	 * \param _fi Pixel residual
	 * \return Weighted residual (weighted by inverse depth uncertainty)
	 */
	types::Float calculatefJ(rebvio::types::EdgeMap::SharedPtr _map, int _f_inx, types::Float& _df_dx, types::Float& _df_dy, rebvio::types::KeyLine& _keyline,
			const types::Float& _px, const types::Float& _py, int& _mnum, types::Float& _fi);

	/**
	 * \brief
	 * \param Score
	 */
	types::Float tryVel(rebvio::types::EdgeMap::SharedPtr _map, rebvio::types::Matrix3f& _JtJ, rebvio::types::Vector3f& _JtF, const rebvio::types::Vector3f& _vel,
			 types::Float _sigma_rho_min, types::Float* _residuals);

	/**
	 * \brief Method to estimate the translational component of the rigid transformation between the input edge map and the edge map used to build the distance field
	 * \param _map Edge map to use for minimization with distance field (which is built with a different edge map)
	 * \param _vel Estimated translation
	 * \param _Rvel Estimated translation uncertainty
	 * \return Minimization score
	 */
	types::Float minimizeVel(rebvio::types::EdgeMap::SharedPtr _map, rebvio::types::Vector3f& _vel, rebvio::types::Matrix3f& _Rvel);

	/**
	 * \brief Method to estimate the full rigid transform (rotation and translation) between the input edge map and the edge map used to build the distance field, using a linear approximation
	 * \param _map Edge map to use for minimization with distance field (which is built with a different edge map)
	 * \param _vel Prior translation estimate
	 * \param _Wx Output information matrix
	 * \param _Rx Inverse of output information matrix
	 * \param _X Output incremental state
	 * \return Success of estimation (true if successful)
	 */
	bool extRotVel(rebvio::types::EdgeMap::SharedPtr _map, const rebvio::types::Vector3f& _vel, rebvio::types::Matrix6f& _Wx, rebvio::types::Matrix6f& _Rx, rebvio::types::Vector6f& _X);

	/**
	 * \brief Method to correct the estimated rigid transform using the gyro prior
	 * \param _X Rigid transform that is to be bias corrected
	 * \param _Wx Estimated and output state information matrix
	 * \param _Gb Estimated and output bias
	 * \param _Wb Estimated and output state information matrix
	 * \param _Rg Gyro measurement covariance matrix
	 * \param _Rb Gyro bias covariance matrix
	 */
	void correctBias(rebvio::types::Vector6f& _X, rebvio::types::Matrix6f& _Wx, rebvio::types::Vector3f& _Gb,
									 rebvio::types::Matrix3f& _Wb, const rebvio::types::Matrix3f& _Rg, const rebvio::types::Matrix3f& _Rb);

	void estimateLs4Acceleration(const rebvio::types::Vector3f& _vel, rebvio::types::Vector3f& _acc,
															 const rebvio::types::Matrix3f& _R, types::Float _dt);

	void estimateMeanAcceleration(const rebvio::types::Vector3f _sacc, rebvio::types::Vector3f& _acc, const rebvio::types::Matrix3f& _R);

	types::Float estimateBias(const rebvio::types::Vector3f& _sacc, const rebvio::types::Vector3f& _facc, types::Float _kP, const rebvio::types::Matrix3f _Rot,
											rebvio::types::Vector7f& _X, rebvio::types::Matrix7f& _P, const rebvio::types::Matrix3f& _Qg, const rebvio::types::Matrix3f& _Qrot,
	                    const rebvio::types::Matrix3f& _Qbias, types::Float _QKp, types::Float _Rg, const rebvio::types::Matrix3f& _Rs,
											const rebvio::types::Matrix3f& _Rf, rebvio::types::Vector3f& _g_est, rebvio::types::Vector3f& _b_est, const rebvio::types::Matrix6f& _Wvw,
											rebvio::types::Vector6f& _Xvw, types::Float _g_gravit);

	/**
	 * \brief Update the inverse depth of the keylines in the current edge map (used to build the distance field) using an EKF
	 * \param _vel Estimated rigid transform translation
	 */
	void updateInverseDepth(rebvio::types::Vector3f& _vel);

	void updateInverseDepthARLU(rebvio::types::KeyLine& _keyline, rebvio::types::Vector3f& _vel);

private:
	EdgeTracker::Config config_;             //!< Configuration parameters
	rebvio::Camera::SharedPtr camera_;       //!< Camera device
	rebvio::DistanceField distance_field_;   //!< Distance field for pose estimation
	unsigned int frame_count_;               //!< Number of frame minimizations performed


};

} /* namespace rebvio */

#endif /* INCLUDE_REBVIO_EDGE_TRACKER_HPP_ */
