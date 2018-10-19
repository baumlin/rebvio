/*
 * scale_space.hpp
 *
 *  Created on: Aug 30, 2018
 *      Author: baumlin
 */

#ifndef INCLUDE_REBVIO_SCALE_SPACE_HPP_
#define INCLUDE_REBVIO_SCALE_SPACE_HPP_

#include "rebvio/camera.hpp"

#include <opencv2/core.hpp>

namespace rebvio {


/**
 * \brief Class that implements "Fast Almost-Gaussian Filtering" (P. Kovesi, 2010)
 * Gaussian filtering is approximated using repeated filtering with averaging filters at a fixed cost per pixel independent of the filter size.
 */
class FastGaussian {
public:
	FastGaussian(rebvio::Camera::SharedPtr _cam, types::Float _sigma, int _n=3);
	FastGaussian() = delete;
	~FastGaussian();

	/**
	 * Smooth the input image using the Fast Almost-Gaussian Method
	 */
	cv::Mat smooth(cv::Mat&);

private:
	/**
	 * Method to calculate the integral image of an input image
	 */
	cv::Mat createIntegralImage(cv::Mat&);
	/**
	 * Method to calculate the average
	 */
	cv::Mat average(cv::Mat&, int, cv::Mat&);
	/**
	 * Method to pre-calculate the divisor at each pixel (number of pixels in the boxfilter) for usage in the average() method
	 */
	void precomputeDivisors(int _d, cv::Mat& _div);

public:
	int n_;										//< Number of averaging operations performed to approximate the Gaussian filter
	types::Float sigma_;			//< Standard deviation of the approximated Gaussian filter
	types::Float sigma_true_;	//< True standard deviation of the approximated Gaussian filter due to rounding errors
	int* widths_;							//< Array containing the widths of the box filters
	cv::Mat* divisors_;				//< Pre-computed divisors used by the average() method
};

/**
 * \brief Class that implements a scale space with two scales and one octave using approximate Gaussian filtering for DoG edge detection
 */
class ScaleSpace {
public:
	ScaleSpace(rebvio::Camera::SharedPtr);
	ScaleSpace() = delete;
	~ScaleSpace();

	/**
	 * \brief Return the Difference of Gaussians calculated via the build() method
	 */
	cv::Mat dog() const;

	/**
	 * \brief Return the squared gradient magnitude of the first scale image calculated via the build() method
	 */
	cv::Mat mag() const;

	/**
	 * \brief Calculate the Difference of Gaussians (using the FastGaussian Filter) and squared gradient magnitude
	 */
	void build(cv::Mat&);


private:
	/**
	 * Method to calculate the Difference of Gaussians
	 */
	void calculateDoG();
	/**
	 * Method to calculate the squared magnitude of the gradient of the first sigma-smoothed image
	 */
	void calculateGradientMagnitude();

private:
	rebvio::Camera::SharedPtr camera_;	//< Camera Device
	rebvio::FastGaussian filter_[2];		//< Array of Gaussian filters with different (increasing) sigmas used to calculate the different scales for the DoG
	cv::Mat scale_[2];									//< Array of the scale images (with increasing sigmas) calculated using the Gaussian filters
	cv::Mat dog_;												//< The Difference of Gaussians calculated using the different scale images
	cv::Mat gradient_mag_; 							//< Squared gradient magnitude of the first scale image
};

} /* namespace rebvio */

#endif /* INCLUDE_REBVIO_SCALE_SPACE_HPP_ */
