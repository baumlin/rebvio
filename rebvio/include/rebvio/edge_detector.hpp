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
#include <opencv2/core.hpp>
#include <rebvio/edge_map.hpp>

namespace rebvio {

struct EdgeDetectorConfig {
	int keylines_ref{12000};                         //!< Number of keylines when autothresholded
	int keylines_max{16000};                         //!< Maximum number of keylines
	static constexpr int plane_fit_size{2};          //!< Window radius for DoG plane fitting: (2*plane_fit_size+1)^2
	types::Float pos_neg_threshold{0.4};             //!< Max percentual difference for DoG nonmaximum suppression
	types::Float dog_threshold{0.095259868922420};   //!< Relation between DoG threshold and gradient threshold ~1/sigma0^4
	types::Float threshold{0.01};                    //!< Manual treshold
	types::Float gain{5e-7};                         //!< Autogain for threshold (if 0, autogain is disabled)
	types::Float max_threshold{0.5};                 //!< Max limit for autothreshold
	types::Float min_threshold{0.005};               //!< Min limit for autothreshold
  static constexpr int num_bins{100};              //!< Number of histogram bins for autothreshold calculation

  using SharedPtr = std::shared_ptr<rebvio::EdgeDetectorConfig>;
};

/**
 * \class EdgeDetector
 *
 * Edge Detection from an input grayscale image according to "Realtime edge-based visual odometry for a monocular camera" (Tarrio & Pedre, 2015), Section 2.1
 */
class EdgeDetector {
public:
	EdgeDetector(rebvio::Camera::SharedPtr _camera, rebvio::EdgeDetectorConfig::SharedPtr _config = std::make_shared<rebvio::EdgeDetectorConfig>());
	EdgeDetector() = delete;
	~EdgeDetector();

	/**
	 * \brief Method to detect edges in the input image and return them as a shared pointer to an edge map
	 * \param _image Image for edge detection
	 * \return Shared pointer to the detected edge map
	 */
	rebvio::EdgeMap::SharedPtr detect(rebvio::types::Image& _image);

private:

	/**
	 * \brief Detects keylines and creates the initial edge map
	 * \param _image Image for edge detection
	 * \return Shared pointer to the detected edge map
	 */
	rebvio::EdgeMap::SharedPtr buildEdgeMap(rebvio::types::Image& _image);

	/**
	 * \brief Joins the edges (sets next and previous ID of the keylines) in the edge map
	 * \param _map Edge map
	 */
	void joinEdges(rebvio::EdgeMap::SharedPtr& _map);

	/**
	 * \brief Returns the index of the next keyline along the direction of the current keyline (perpendicular to its gradient)
	 * \param _map Edge map
	 * \param _x Integer x coordinate of the current keyline
	 * \param _y Integer y coordinate of the current keyline
	 * \param _idx Index of the current keyline
	 * \return Index of the next keyline
	 */
	int nextKeylineIdx(rebvio::EdgeMap::SharedPtr& _map, int _x, int _y, int _idx);

	/**
	 * \brief Auto-threshold calculation based on the histogram of DoG values
	 * \param _map Edge map
	 */
	void tuneThreshold(rebvio::EdgeMap::SharedPtr _map);

private:
	EdgeDetectorConfig::SharedPtr config_;  //!< Configuration parameters
	int keylines_count_;                    //!< Current number of detected keylines in current image
	cv::Mat keylines_mask_;                 //!< Image mask containing the keyline IDs (-1 if no keyline)

	rebvio::Camera::SharedPtr camera_;      //!< Camera Device
	rebvio::ScaleSpace scale_space_;        //!< Scale space for edge detection

	types::Float auto_threshold_;           //!< Auto threshold
	int max_image_value_;                   //!< Max input image value for auto threshold calculation
};

} /* namespace rebvio */

#endif /* INCLUDE_REBVIO_EDGE_DETECTOR_HPP_ */
