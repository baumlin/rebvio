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
	EdgeDetector(rebvio::CameraPtr);
	~EdgeDetector();

	rebvio::types::EdgeMapPtr detect(rebvio::types::Image&,int);

	int getNumKeylines() const;
	cv::Mat& getMask();

private:
	void buildMask(rebvio::types::EdgeMapPtr&);
	void joinEdges(rebvio::types::EdgeMapPtr&);
	int nextPoint(rebvio::types::EdgeMapPtr&,int,int,int);
	void tuneThreshold(rebvio::types::EdgeMapPtr,int);

private:
	int keylines_count_;							//< Current number of detected keylines in current image
	int keylines_size_;								//<
	cv::Mat keylines_mask_;						//< Image mask containing the keyline IDs (-1 if no keyline)
	int points_ref_;									//< Number of points when autothresholded
	int points_max_;									//< Maximum number of points
	int points_tracked_;

	rebvio::CameraPtr camera_;
	rebvio::ScaleSpace scale_space_;

	int plane_fit_size_;							//< Half window size -1 for DoG plane fitting: (2*plane_fit_size+1)^2
	float pos_neg_threshold_;					//< Max percentual difference for DoG nonmaximum suppression
	float dog_threshold_;							//< Relation between DoG threshold and gradient threshold ~1/sigma0^4
	float threshold_;									//< Manual treshold
	float tuned_threshold_;
	float gain_;											//< Autogain for threshold (if 0, autogain is disabled)
	float max_threshold_;							//< Max limit for autothreshold
	float min_threshold_;							//< Min limit for autothreshold
	int max_image_value_;
};

} /* namespace rebvio */

#endif /* INCLUDE_REBVIO_EDGE_DETECTOR_HPP_ */
