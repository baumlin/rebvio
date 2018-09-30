/*
 * EdgeDetector.hpp
 *
 *  Created on: Aug 29, 2018
 *      Author: baumlin
 */

#ifndef INCLUDE_REBVIO_EDGE_DETECTOR_HPP_
#define INCLUDE_REBVIO_EDGE_DETECTOR_HPP_

#include "rebvio/scale_space.hpp"
#include "rebvio/types/keyline.hpp"
#include "rebvio/types/image.hpp"
#include "rebvio/types/edge_map.hpp"

#include <opencv2/core.hpp>

namespace rebvio {

class EdgeDetector {
public:
	EdgeDetector(rebvio::Camera::SharedPtr);
	~EdgeDetector();

	rebvio::types::EdgeMap::SharedPtr detect(rebvio::types::Image&,int);

	int getNumKeylines() const;
	cv::Mat& getMask();

private:
	void buildMask(rebvio::types::EdgeMap::SharedPtr&);
	void joinEdges(rebvio::types::EdgeMap::SharedPtr&);
	int nextPoint(rebvio::types::EdgeMap::SharedPtr&,int,int,int);
	void tuneThreshold(rebvio::types::EdgeMap::SharedPtr,int);

private:
	int keylines_count_;							//< Current number of detected keylines in current image
	cv::Mat keylines_mask_;						//< Image mask containing the keyline IDs (-1 if no keyline)
	int points_ref_;									//< Number of points when autothresholded
	int points_max_;									//< Maximum number of points

	rebvio::Camera::SharedPtr camera_;
	rebvio::ScaleSpace scale_space_;

	int plane_fit_size_;							//< Half window size -1 for DoG plane fitting: (2*plane_fit_size+1)^2
	types::Float pos_neg_threshold_;					//< Max percentual difference for DoG nonmaximum suppression
	types::Float dog_threshold_;							//< Relation between DoG threshold and gradient threshold ~1/sigma0^4
	types::Float threshold_;									//< Manual treshold
	types::Float tuned_threshold_;
	types::Float gain_;											//< Autogain for threshold (if 0, autogain is disabled)
	types::Float max_threshold_;							//< Max limit for autothreshold
	types::Float min_threshold_;							//< Min limit for autothreshold
	int max_image_value_;
};

} /* namespace rebvio */

#endif /* INCLUDE_REBVIO_EDGE_DETECTOR_HPP_ */
