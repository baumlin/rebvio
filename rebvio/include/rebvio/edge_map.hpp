/*
 * edge_map.hpp
 *
 *  Created on: Sep 2, 2018
 *      Author: baumlin
 */

#ifndef INCLUDE_REBVIO_EDGE_MAP_HPP_
#define INCLUDE_REBVIO_EDGE_MAP_HPP_

#include <memory>
#include <vector>
#include <unordered_map>
#include "rebvio/types/keyline.hpp"
#include "rebvio/types/imu.hpp"

namespace rebvio {

struct EdgeMapConfig {
	types::Float pixel_uncertainty_match{2.0};       //!< Pixel uncertainty for the matching step
	types::Float match_threshold_norm{1.0};          //!< Relative norm threshold between two keyline gradients for keyline matching
	types::Float match_threshold_angle{45.0};        //!< Angle threshold [deg] between two keyline gradients for keyline matching
	types::Float regularization_threshold{0.5};      //!< Edgemap regularization threshold on angle beta between neighboring keyline gradients (threshold = cos(beta))

	using SharedPtr = std::shared_ptr<rebvio::EdgeMapConfig>;
};

class EdgeMap {

public:
	using SharedPtr = std::shared_ptr<rebvio::EdgeMap>;

public:
	EdgeMap(rebvio::Camera::SharedPtr _camera, int _size, uint64_t _ts_us, rebvio::EdgeMapConfig::SharedPtr _config = std::make_shared<rebvio::EdgeMapConfig>());
	EdgeMap() = delete;

	/**
	 * \brief Return a reference to the keyline at index _idx in the keylines_ vector
	 */
	rebvio::types::KeyLine& operator[](int _idx);

	/**
	 * \brief Return the size of the map, i.e. the number of keylines
	 */
	int size();

	/**
	 * \brief Return a reference to the keylines vector
	 */
	std::vector<rebvio::types::KeyLine>& keylines();

	/**
	 * \brief Return the timestamp of the image in [us], in which the edge map was detected
	 */
	uint64_t ts_us();

	/**
	 * \brief Return a reference to the threshold used for keyline detection of the edge map
	 */
	const types::Float& threshold() const;

	/**
	 * \brief Sets the treshold used for keyline detection of the edge map
	 */
	void threshold(const types::Float& _treshold);

	/**
	 * \brief Returns a reference to the integrated IMU data since the last edge map detection
	 */
	rebvio::types::IntegratedImu& imu();

	/**
	 * \brief Return a reference to image index <-> keyline index lookup table
	 */
	std::unordered_map<unsigned int,unsigned int>& mask();

	/**
	 * \brief Estimate the Sigma-Rho quantile according to _percentile fraction of the number of keylines
	 */
	types::Float estimateQuantile(types::Float _percentile, int _num_bins);

	/**
	 * \brief Rotate the keylines in the edge map according to the Rotation Matrix _R
	 */
	void rotateKeylines(const rebvio::types::Matrix3f& _R);

	/**
	 * \brief Search keyline matches of this map with keylines in _map
	 */
	int forwardMatch(rebvio::EdgeMap::SharedPtr _map);

	/**
	 * \brief Search for a match of _keyline (from another map) in this map
	 */
	int searchMatch(const rebvio::types::KeyLine& _keyline, const rebvio::types::Vector3f& _vel, const rebvio::types::Matrix3f& _Rvel,
									const rebvio::types::Matrix3f& _Rback, types::Float _max_radius);

	/**
	 * \brief Search keyline matches of this map with keylines in _map, given additional information regarding the extrinsics between the maps
	 */
	int directedMatch(rebvio::EdgeMap::SharedPtr _map, const rebvio::types::Vector3f& _vel, const rebvio::types::Matrix3f& _Rvel,
			              const rebvio::types::Matrix3f& _Rback, int& _kf_matches, types::Float _max_radius);

	/**
	 * \brief Regularize the inverse depth quantities (rho, sigma_rho) of this edge map using a weighted mean of the depths of the two neighbours of a keyline
	 * \param _threshold Angular threshold: _threshold = cos(beta), where beta is the angle between the gradients of the two neightbours of a keyline
	 * \return Number or regularized keylines
	 */
	int regularize1Iter();

private:

	/**
	 * \brief Convert row-col pixel position into linear index (Row-Major with origin in upper left corner)
	 * \param _row y-coordinate of position
	 * \param _col x-coordinate of position
	 * \return Linear index according to input position
	 */
	inline int getIndex(types::Float _row, types::Float _col) {
		int row = std::round(_row);
		int col = std::round(_col);
		if(row >= camera_->rows_ || row < 0 || col >= camera_->cols_ || col < 0) return -1;
		return row*camera_->cols_+col;
	}

private:
	rebvio::EdgeMapConfig::SharedPtr config_;                     //!< Configuration parameters
	rebvio::Camera::SharedPtr camera_;														//!< Camera Device
	uint64_t ts_us_;																							//!< Timestamp in [us]
	std::vector<rebvio::types::KeyLine> keylines_;								//!< Vector of keylines in the edge map
	std::unordered_map<unsigned int,unsigned int> keylines_mask_; //!< A lookup table with the image index as key and keyline index in keylines_ as value
	types::Float threshold_; 																			//!< Threshold that was used for keyline detection
	rebvio::types::IntegratedImu imu_;														//!< Integrated Imu data since last edge map
	unsigned int matches_;																				//!< Number of matches with other edge map
};

} /* namespace rebvio */

#endif /* INCLUDE_REBVIO_EDGE_MAP_HPP_ */
