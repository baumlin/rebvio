/*
 * scale_space.cpp
 *
 *  Created on: Aug 30, 2018
 *      Author: baumlin
 */

#include "rebvio/scale_space.hpp"
#include "rebvio/util/timer.hpp"
#include <cmath>

namespace rebvio {

FastGaussian::FastGaussian(Camera::SharedPtr _camera, types::Float _sigma, int _n) :
		n_(_n),
		sigma_(_sigma),
		widths_(new int[n_])
{
	// compute the ideal width of the averaging filter (3)
	types::Float w_ideal = sqrt(12.0*sigma_*sigma_/types::Float(n_+1));
	int w_l = int(w_ideal);
	if(int(w_l/2)*2==w_l) --w_l; // w_l should be first odd integer less than w_ideal

	// m is the number of passes with a filter of width w_l followed by (n_-m) passes with a filter of width w_u, where 0<=m<=n, to achieve the filtering (5)
	int m = std::round((3*n_+4*n_*w_l+n_*w_l*w_l-12*sigma_*sigma_)/(4+4*w_l));
	int i;
	for(i=0;i<m;i++){
		widths_[i]=w_l; // applying an averaging filter of width w_l m times
	}
	for(;i<n_;i++){
		widths_[i]=w_l+2; // applying an averaging filter of width w_u = w_l+2 (n_-m) times
	}

	// sigma_true_ is the effective standard deviation due to the rounding error of m (4)
	sigma_true_=sqrt((m*w_l*w_l+(n_-m)*(w_l+2.0)*(w_l+2.0)-n_)/12.0);
	divisors_ = new cv::Mat[n_];
	for(int i=0;i<n_;i++){
		divisors_[i]=cv::Mat(_camera->rows_,_camera->cols_,CV_FLOAT_PRECISION);
		precomputeDivisors(widths_[i],divisors_[i]);
	}
}

FastGaussian::~FastGaussian() {
	if(divisors_) delete[] divisors_;
	if(widths_) delete[] widths_;
}

cv::Mat FastGaussian::createIntegralImage(cv::Mat& _in) {
	cv::Mat integral_image = cv::Mat(_in.rows,_in.cols,CV_FLOAT_PRECISION);
	for(int row = 0; row < _in.rows; ++row) {
		types::Float* ii_ptr = integral_image.ptr<types::Float>(row);
		const types::Float* in_ptr = _in.ptr<types::Float>(row);
		ii_ptr[0] = in_ptr[0];
		for(int col = 1; col < _in.cols; ++col) {
			ii_ptr[col] = ii_ptr[col-1]+in_ptr[col];
		}
	}

	for(int row = 1; row < integral_image.rows; ++row) {
		const types::Float* ii_ptr_prev = integral_image.ptr<types::Float>(row-1);
		types::Float* ii_ptr = integral_image.ptr<types::Float>(row);
		for(int col = 0; col < integral_image.cols; ++col) {
			ii_ptr[col] += ii_ptr_prev[col];
		}
	}
	return integral_image;
}

cv::Mat FastGaussian::average(cv::Mat& _integral_image, int _d, cv::Mat& _div) {
	cv::Mat average_image = cv::Mat(_integral_image.rows,_integral_image.cols,CV_FLOAT_PRECISION);
	int d2 = _d/2;
	types::Float a = 1.0/(_d*_d);
	for(int row = 0; row < d2+1; ++row) {
		const types::Float* div_ptr = _div.ptr<types::Float>(row);
		const types::Float* ii_ptr = _integral_image.ptr<types::Float>(row+d2);
		types::Float* ai_ptr = average_image.ptr<types::Float>(row);
		for(int col = 0; col < d2+1; ++col) {
			ai_ptr[col] = ii_ptr[col+d2]*div_ptr[col];
		}
		for(int col = d2+1; col < _div.cols-d2; ++col) {
			ai_ptr[col] = (ii_ptr[col+d2]-ii_ptr[col-d2-1])*div_ptr[col];
		}
		for(int col = _div.cols-d2; col < _div.cols; ++col) {
			ai_ptr[col] = (ii_ptr[_div.cols-1]-ii_ptr[col-d2-1])*div_ptr[col];
		}
	}
	for(int row = d2+1; row < _div.rows-d2; ++row) {
		const types::Float* div_ptr = _div.ptr<types::Float>(row);
		const types::Float* ii_ptr1 = _integral_image.ptr<types::Float>(row+d2);
		const types::Float* ii_ptr2 = _integral_image.ptr<types::Float>(row-d2-1);
		types::Float* ai_ptr = average_image.ptr<types::Float>(row);
		for(int col = 0; col < d2+1; ++col) {
			int col_idx = col+d2;
			ai_ptr[col] = (ii_ptr1[col_idx]-ii_ptr2[col_idx])*div_ptr[col];
		}
		for(int col = d2+1; col < _div.cols-d2; ++col) {
			int col_idx1 = col+d2;
			int col_idx2 = col-d2-1;
			ai_ptr[col] = (ii_ptr1[col_idx1]-ii_ptr1[col_idx2] - ii_ptr2[col_idx1]+ii_ptr2[col_idx2])*a;
		}
		for(int col = _div.cols-d2; col < _div.cols; ++col) {
			int col_idx1 = _div.cols-1;
			int col_idx2 = col-d2-1;
			ai_ptr[col] = (ii_ptr1[col_idx1]-ii_ptr1[col_idx2] - ii_ptr2[col_idx1]+ii_ptr2[col_idx2])*div_ptr[col];
		}
	}
	for(int row = _div.rows-d2; row < _div.rows; ++row) {
		const types::Float* div_ptr = _div.ptr<types::Float>(row);
		const types::Float* ii_ptr1 = _integral_image.ptr<types::Float>(_div.rows-1);
		const types::Float* ii_ptr2 = _integral_image.ptr<types::Float>(row-d2-1);
		types::Float* ai_ptr = average_image.ptr<types::Float>(row);
		for(int col = 0; col < d2+1; ++col) {
			int col_idx = col+d2;
			ai_ptr[col] = (ii_ptr1[col_idx]-ii_ptr2[col_idx])*div_ptr[col];
		}
		for(int col = d2+1; col < _div.cols-d2; ++col) {
			int col_idx1 = col+d2;
			int col_idx2 = col-d2-1;
			ai_ptr[col] = (ii_ptr1[col_idx1]-ii_ptr2[col_idx1] - ii_ptr1[col_idx2]+ii_ptr2[col_idx2])*div_ptr[col];
		}
		for(int col = _div.cols-d2; col < _div.cols; ++col) {
			int col_idx1 = _div.cols-1;
			int col_idx2 = col-d2-1;
			ai_ptr[col] = (ii_ptr1[col_idx1]-ii_ptr2[col_idx1] - ii_ptr1[col_idx2]+ii_ptr2[col_idx2])*div_ptr[col];
		}
	}
	return average_image;
}

void FastGaussian::precomputeDivisors(int _d, cv::Mat& _div) {
	int d2 = _d/2;
	types::Float a = (types::Float)_d*_d;
	for(int row = 0; row < d2+1; ++row) {
		for(int col = 0; col < d2+1; ++col) {
			_div.at<types::Float>(row,col) = (col+d2+1)*(row+d2+1);
		}
		for(int col = d2+1; col < _div.cols-d2; ++col) {
			_div.at<types::Float>(row,col) = (types::Float)_d*(row+d2+1);
		}
		for(int col = _div.cols-d2; col < _div.cols; ++col) {
			_div.at<types::Float>(row,col) = (_div.cols-col+d2)*(row+d2+1);
		}
	}
	for(int row = d2+1; row < _div.rows-d2; ++row) {
		for(int col = 0; col < d2+1; ++col) {
			_div.at<types::Float>(row,col) = (col+d2+1)*(types::Float)_d;
		}
		for(int col = d2+1; col < _div.cols-d2; ++col) {
			_div.at<types::Float>(row,col) = a;
		}
		for(int col = _div.cols-d2; col < _div.cols; ++col) {
			_div.at<types::Float>(row,col) = (_div.cols-col+d2)*(types::Float)_d;
		}
	}
	for(int row = _div.rows-d2; row < _div.rows; ++row) {
		for(int col = 0; col < d2+1; ++col) {
			_div.at<types::Float>(row,col) = (_div.rows-row+d2)*(col+d2+1);
		}
		for(int col = d2+1; col < _div.cols-d2; ++col) {
			_div.at<types::Float>(row,col) = (_div.rows-row+d2)*(types::Float)_d;
		}
		for(int col = _div.cols-d2; col < _div.cols; ++col) {
			_div.at<types::Float>(row,col) = (_div.rows-row+d2)*(_div.cols-col+d2);
		}
	}
	for(int row = 0; row < _div.rows; ++row) {
		for(int col = 0; col < _div.cols; ++col) {
			_div.at<types::Float>(row,col) = 1.0/_div.at<types::Float>(row,col);
		}
	}
}

cv::Mat FastGaussian::smooth(cv::Mat& _in) {
	cv::Mat out;
	cv::Mat integral_image = createIntegralImage(_in);
	for(int i = 0; i < n_-1; ++i) {
		out = average(integral_image,widths_[i],divisors_[i]);
		integral_image = createIntegralImage(out);
	}
	out = average(integral_image,widths_[n_-1],divisors_[n_-1]);
	return out;
}

ScaleSpace::ScaleSpace(rebvio::Camera::SharedPtr _camera) :
		camera_(_camera),
		filter_{rebvio::FastGaussian(camera_,3.56359,3),rebvio::FastGaussian(camera_,filter_[0].sigma_true_*1.2599,3)}
{
		dog_ = cv::Mat(camera_->rows_,camera_->cols_,CV_FLOAT_PRECISION,cv::Scalar(0.0));
		gradient_mag_ = cv::Mat(camera_->rows_,camera_->cols_,CV_FLOAT_PRECISION,cv::Scalar(0.0));
}

ScaleSpace::~ScaleSpace() {
}

cv::Mat ScaleSpace::dog() const {
	return dog_;
}

cv::Mat ScaleSpace::mag() const {
	return gradient_mag_;
}

void ScaleSpace::build(cv::Mat& _image) {
	scale_[0] = filter_[0].smooth(_image);
	scale_[1] = filter_[1].smooth(_image);
	calculateDoG();
	calculateGradientMagnitude();
}

void ScaleSpace::calculateDoG() {
	for(int row = 0; row < dog_.rows; ++row) {
		types::Float* dog_ptr = dog_.ptr<types::Float>(row);
		const types::Float* i0_ptr  = scale_[0].ptr<types::Float>(row);
		const types::Float* i1_ptr  = scale_[1].ptr<types::Float>(row);
		for(int col = 0; col < dog_.cols; ++col) {
			dog_ptr[col] = i1_ptr[col]-i0_ptr[col]; // LoG is approximated with the difference of the larger sigma s1 and the smaller sigma s0: s1-s0
		}
	}
}

void ScaleSpace::calculateGradientMagnitude() {
	for(int row = 1; row < gradient_mag_.rows-1; ++row) {
		const types::Float* ic_ptr = scale_[0].ptr<types::Float>(row);
		const types::Float* irl_ptr = scale_[0].ptr<types::Float>(row-1);
		const types::Float* iru_ptr = scale_[0].ptr<types::Float>(row+1);
		types::Float* mag_ptr = gradient_mag_.ptr<types::Float>(row);
		for(int col = 1; col < gradient_mag_.cols-1; ++col) {
			types::Float dx = ic_ptr[col+1]-ic_ptr[col-1];
			types::Float dy = iru_ptr[col]-irl_ptr[col];
			mag_ptr[col] = dx*dx+dy*dy; // squared magnitude of the image gradient in the first scale
		}
	}
}

} /* namespace rebvio */
