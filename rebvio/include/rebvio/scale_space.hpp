/*
 * scale_space.hpp
 *
 *  Created on: Aug 30, 2018
 *      Author: baumlin
 */

#ifndef INCLUDE_REBVIO_SCALE_SPACE_HPP_
#define INCLUDE_REBVIO_SCALE_SPACE_HPP_

#include "rebvio/camera.hpp"
#include "rebvio/types/image.hpp"

#include <opencv2/core.hpp>
#include <eigen3/Eigen/Dense>

namespace rebvio {


typedef Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> EigenMat;
/**
 * @brief Class that implements "Fast Almost-Gaussian Filtering" (P. Kovesi, 2010)
 * Gaussian filtering is approximated using repeated filtering with averaging filters at a fixed cost per pixel independent of the filter size.
 */
class FastGaussian {
public:
	FastGaussian(rebvio::Camera _cam, float _sigma, int _n=3);
	~FastGaussian();

	/**
	 * Smooth the input image using the Fast Almost-Gaussian Method
	 */
	cv::Mat smooth(cv::Mat&);
	EigenMat smooth(EigenMat&&);

private:
	/**
	 * Method to calculate the integral image of an input image
	 */
	cv::Mat createIntegralImage(cv::Mat&);
	EigenMat createIntegralImage(EigenMat&);
	/**
	 * Method to calculate the average
	 */
	cv::Mat average(cv::Mat&, int, cv::Mat&);
	EigenMat average(EigenMat&, int, Eigen::MatrixXf&);
	/**
	 * Method to pre-calculate the divisor at each pixel (number of pixels in the boxfilter) for usage in the average() method
	 */
	void precomputeDivisors(int _d, cv::Mat& _div);
	void precomputeDivisors(int _d, Eigen::MatrixXf& _div);

public:
	int n_;							//< Number of averagings performed to approximate the Gaussian filter
	float sigma_;				//< Standard deviation of the approximated Gaussian filter
	float sigma_true_;	//< True standard deviation of the approximated Gaussian filter due to rounding errors
	int* widths_;				//< Array containing the widths of the box filters
	cv::Mat* divisors_;	//< Pre-computed divisors used by the average() method
	Eigen::MatrixXf* divisors_e_;
};

/**
 * @brief Class that implements a scale space with two scales using approximate Gaussian filtering for edge detection
 */
class ScaleSpace {
public:
	ScaleSpace();
	~ScaleSpace();
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

public:
	rebvio::Camera camera_;
	rebvio::FastGaussian filter_[2];//< Array of Gaussian filters with different (increasing) sigmas used to calculate the different scales for the DoG
	cv::Mat scale_[2];							//< Array of the scale images (with increasing sigmas) calculated using the Gaussian filters
	EigenMat scale_e_[2];
	cv::Mat dog_;										//< The Difference of Gaussians calculated using the different scale images
	cv::Mat gradient_mag_; 					//< Squared gradient magnitude of the first scale image
};

} /* namespace rebvio */

#endif /* INCLUDE_REBVIO_SCALE_SPACE_HPP_ */
