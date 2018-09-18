/*
 * scale_space.cpp
 *
 *  Created on: Aug 30, 2018
 *      Author: baumlin
 */

#include "rebvio/scale_space.hpp"
#include "rebvio/util/timer.hpp"
#include <cmath>
#include <iostream>

namespace rebvio {

FastGaussian::FastGaussian(Camera _camera, float _sigma,int _n) :
		n_(_n),
		sigma_(_sigma),
		widths_(new int[n_])
{
	// compute the ideal width of the averaging filter (3)
	float w_ideal = sqrt(12.0*sigma_*sigma_/float(n_+1));
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
	divisors_e_ = new Eigen::MatrixXf[n_];
	for(int i=0;i<n_;i++){
		divisors_[i]=cv::Mat(_camera.rows_,_camera.cols_,CV_32FC1);
		divisors_e_[i] = Eigen::MatrixXf(_camera.rows_,_camera.cols_);
		precomputeDivisors(widths_[i],divisors_[i]);
		precomputeDivisors(widths_[i],divisors_e_[i]);
	}
}

FastGaussian::~FastGaussian() {
	if(divisors_) delete[] divisors_;
	if(divisors_e_) delete[] divisors_e_;
	if(widths_) delete[] widths_;
}

cv::Mat FastGaussian::createIntegralImage(cv::Mat& _in) {
	cv::Mat integral_image = cv::Mat(_in.rows,_in.cols,CV_32FC1);
	for(int row = 0; row < _in.rows; ++row) {
		float* ii_ptr = integral_image.ptr<float>(row);
		const float* in_ptr = _in.ptr<float>(row);
		ii_ptr[0] = in_ptr[0];
		for(int col = 1; col < _in.cols; ++col) {
			ii_ptr[col] = ii_ptr[col-1]+in_ptr[col];
		}

//		integral_image.at<float>(row,0) = _in.at<float>(row,0);
//		for(int col = 1; col < _in.cols; ++col) {
//			integral_image.at<float>(row,col) = integral_image.at<float>(row,col-1)+_in.at<float>(row,col);
//		}
	}

	for(int row = 1; row < integral_image.rows; ++row) {
		const float* ii_ptr_prev = integral_image.ptr<float>(row-1);
		float* ii_ptr = integral_image.ptr<float>(row);
		for(int col = 0; col < integral_image.cols; ++col) {
			ii_ptr[col] += ii_ptr_prev[col];
		}
	}

//	for(int col = 0; col < integral_image.cols; ++col) {
//		for(int row = 1; row < integral_image.rows; ++row) {
//			integral_image.at<float>(row,col) += integral_image.at<float>(row-1,col);
//		}
//	}
	return integral_image;
}

EigenMat FastGaussian::createIntegralImage(EigenMat& _in) {
	EigenMat integral_image = EigenMat(_in.rows(),_in.cols());
	integral_image.col(0) = _in.col(0);
	integral_image.block(0,1,_in.rows(),_in.cols()-1) = integral_image.block(0,0,_in.rows(),_in.cols()-1) + _in.block(0,1,_in.rows(),_in.cols()-1);

	for(int row = 1; row < integral_image.rows(); ++row) {
		integral_image.row(row) += integral_image.row(row-1);
	}

	return integral_image;
}

cv::Mat FastGaussian::average(cv::Mat& _integral_image, int _d, cv::Mat& _div) {
	cv::Mat average_image = cv::Mat(_integral_image.rows,_integral_image.cols,CV_32FC1);
	int d2 = _d/2;
	float a = 1.0/(_d*_d);
	for(int row = 0; row < d2+1; ++row) {
		const float* div_ptr = _div.ptr<float>(row);
		const float* ii_ptr = _integral_image.ptr<float>(row+d2);
		float* ai_ptr = average_image.ptr<float>(row);
		for(int col = 0; col < d2+1; ++col) {
//			average_image.at<float>(row,col) = _integral_image.at<float>(row+d2,col+d2)*_div.at<float>(row,col);
			ai_ptr[col] = ii_ptr[col+d2]*div_ptr[col];
		}
		for(int col = d2+1; col < _div.cols-d2; ++col) {
//			average_image.at<float>(row,col) = (_integral_image.at<float>(row+d2,col+d2)-_integral_image.at<float>(row+d2,col-d2-1))*_div.at<float>(row,col);
			ai_ptr[col] = (ii_ptr[col+d2]-ii_ptr[col-d2-1])*div_ptr[col];
		}
		for(int col = _div.cols-d2; col < _div.cols; ++col) {
//			average_image.at<float>(row,col) = (_integral_image.at<float>(row+d2,_div.cols-1)-_integral_image.at<float>(row+d2,col-d2-1))*_div.at<float>(row,col);
			ai_ptr[col] = (ii_ptr[_div.cols-1]-ii_ptr[col-d2-1])*div_ptr[col];
		}
	}
	for(int row = d2+1; row < _div.rows-d2; ++row) {
		const float* div_ptr = _div.ptr<float>(row);
		const float* ii_ptr1 = _integral_image.ptr<float>(row+d2);
		const float* ii_ptr2 = _integral_image.ptr<float>(row-d2-1);
		float* ai_ptr = average_image.ptr<float>(row);
		for(int col = 0; col < d2+1; ++col) {
//			average_image.at<float>(row,col) = (_integral_image.at<float>(row+d2,col+d2)-_integral_image.at<float>(row-d2-1,col+d2))*_div.at<float>(row,col);
			int col_idx = col+d2;
			ai_ptr[col] = (ii_ptr1[col_idx]-ii_ptr2[col_idx])*div_ptr[col];
		}
		for(int col = d2+1; col < _div.cols-d2; ++col) {
//			average_image.at<float>(row,col) = (_integral_image.at<float>(row+d2,col+d2)-_integral_image.at<float>(row+d2,col-d2-1)
//					-_integral_image.at<float>(row-d2-1,col+d2)+_integral_image.at<float>(row-d2-1,col-d2-1))*a;
			int col_idx1 = col+d2;
			int col_idx2 = col-d2-1;
			ai_ptr[col] = (ii_ptr1[col_idx1]-ii_ptr1[col_idx2] - ii_ptr2[col_idx1]+ii_ptr2[col_idx2])*a;
		}
		for(int col = _div.cols-d2; col < _div.cols; ++col) {
//			average_image.at<float>(row,col) = (_integral_image.at<float>(row+d2,_div.cols-1)-_integral_image.at<float>(row+d2,col-d2-1)
//					-_integral_image.at<float>(row-d2-1,_div.cols-1)+_integral_image.at<float>(row-d2-1,col-d2-1))*_div.at<float>(row,col);
			int col_idx1 = _div.cols-1;
			int col_idx2 = col-d2-1;
			ai_ptr[col] = (ii_ptr1[col_idx1]-ii_ptr1[col_idx2] - ii_ptr2[col_idx1]+ii_ptr2[col_idx2])*div_ptr[col];
		}
	}
	for(int row = _div.rows-d2; row < _div.rows; ++row) {
		const float* div_ptr = _div.ptr<float>(row);
		const float* ii_ptr1 = _integral_image.ptr<float>(_div.rows-1);
		const float* ii_ptr2 = _integral_image.ptr<float>(row-d2-1);
		float* ai_ptr = average_image.ptr<float>(row);
		for(int col = 0; col < d2+1; ++col) {
//			average_image.at<float>(row,col) = (_integral_image.at<float>(_div.rows-1,col+d2)-_integral_image.at<float>(row-d2-1,col+d2))*_div.at<float>(row,col);
			int col_idx = col+d2;
			ai_ptr[col] = (ii_ptr1[col_idx]-ii_ptr2[col_idx])*div_ptr[col];
		}
		for(int col = d2+1; col < _div.cols-d2; ++col) {
//			average_image.at<float>(row,col) = (_integral_image.at<float>(_div.rows-1,col+d2)-_integral_image.at<float>(row-d2-1,col+d2)
//					-_integral_image.at<float>(_div.rows-1,col-d2-1)+_integral_image.at<float>(row-d2-1,col-d2-1))*_div.at<float>(row,col);
			int col_idx1 = col+d2;
			int col_idx2 = col-d2-1;
			ai_ptr[col] = (ii_ptr1[col_idx1]-ii_ptr2[col_idx1] - ii_ptr1[col_idx2]+ii_ptr2[col_idx2])*div_ptr[col];
		}
		for(int col = _div.cols-d2; col < _div.cols; ++col) {
//			average_image.at<float>(row,col) = (_integral_image.at<float>(_div.rows-1,_div.cols-1)-_integral_image.at<float>(row-d2-1,_div.cols-1)
//					-_integral_image.at<float>(_div.rows-1,col-d2-1)+_integral_image.at<float>(row-d2-1,col-d2-1))*_div.at<float>(row,col);
			int col_idx1 = _div.cols-1;
			int col_idx2 = col-d2-1;
			ai_ptr[col] = (ii_ptr1[col_idx1]-ii_ptr2[col_idx1] - ii_ptr1[col_idx2]+ii_ptr2[col_idx2])*div_ptr[col];
		}
	}
	return average_image;
}

EigenMat FastGaussian::average(EigenMat& _integral_image, int _d, Eigen::MatrixXf& _div) {
	EigenMat average_image = EigenMat(_integral_image.rows(),_integral_image.cols());
	int d2 = _d/2;
	float a = 1.0/(_d*_d);

	average_image.block(0,0,d2+1,d2+1) = _integral_image.block(0,d2,d2+1,d2+1).cwiseProduct(_div.block(0,0,d2+1,d2+1));
	average_image.block(0,d2+1,d2+1,_div.cols()-d2-d2-1) =
			(_integral_image.block(0,d2+1+d2,d2+1,_div.cols()-d2-d2-1)
					-_integral_image.block(0,d2+1-d2-1,d2+1,_div.cols()-d2-d2-1)).cwiseProduct(_div.block(0,d2+1,d2+1,_div.cols()-d2-d2-1));
	average_image.block(0,_div.cols()-d2,d2+1,_div.cols()-_div.cols()+d2) =
			(_integral_image.block(0,_div.cols()-d2,d2+1,1).replicate(1,_div.cols()-_div.cols()+d2)
					-_integral_image.block(0,_div.cols()-d2-d2-1,d2+1,_div.cols()-_div.cols()+d2)).cwiseProduct(_div.block(0,_div.cols()-d2,d2+1,_div.cols()-_div.cols()+d2));
//	for(int row = 0; row < d2+1; ++row) {
//		const float* div_ptr = _div.ptr<float>(row);
//		const float* ii_ptr = _integral_image.ptr<float>(row+d2);
//		float* ai_ptr = average_image.ptr<float>(row);
//		for(int col = 0; col < d2+1; ++col) {
//			ai_ptr[col] = ii_ptr[col+d2]*div_ptr[col];
//		}
//		for(int col = d2+1; col < _div.cols-d2; ++col) {
//			ai_ptr[col] = (ii_ptr[col+d2]-ii_ptr[col-d2-1])*div_ptr[col];
//		}
//		for(int col = _div.cols-d2; col < _div.cols; ++col) {
//			ai_ptr[col] = (ii_ptr[_div.cols-1]-ii_ptr[col-d2-1])*div_ptr[col];
//		}
//	}

	average_image.block(d2+1,0,_div.rows()-d2-d2-1,d2+1) =
			(_integral_image.block(d2+1+d2,d2,_div.rows()-d2-d2-1,d2+1)
					-_integral_image.block(d2+1-d2-1,d2,_div.rows()-d2-d2-1,d2+1)).cwiseProduct(_div.block(d2+1,0,_div.rows()-d2-d2-1,d2+1));
	average_image.block(d2+1,d2+1,_div.rows()-d2-d2-1,_div.cols()-d2-d2-1) =
			(_integral_image.block(d2+1+d2,d2+1+d2,_div.rows()-d2-d2-1,_div.cols()-d2-d2-1)
					-_integral_image.block(d2+1+d2,d2+1-d2-1,_div.rows()-d2-d2-1,_div.cols()-d2-d2-1)
					- _integral_image.block(d2+1-d2-1,d2+1+d2,_div.rows()-d2-d2-1,_div.cols()-d2-d2-1)
					+_integral_image.block(d2+1-d2-1,d2+1-d2-1,_div.rows()-d2-d2-1,_div.cols()-d2-d2-1))*a;
	average_image.block(d2+1,_div.cols()-d2,_div.rows()-d2-d2-1,_div.cols()-_div.cols()+d2) =
			(_integral_image.block(d2+1+d2,_div.cols()-1,_div.rows()-d2-d2-1,1).replicate(1,_div.cols()
					-_div.cols()+d2)-_integral_image.block(d2+1+d2,_div.cols()-d2-d2-1,_div.rows()-d2-d2-1,_div.cols()-_div.cols()+d2)
					- _integral_image.block(d2+1-d2-1,_div.cols()-1,_div.rows()-d2-d2-1,1).replicate(1,_div.cols()-_div.cols()+d2)
					+_integral_image.block(d2+1-d2-1,_div.cols()-d2-d2-1,_div.rows()-d2-d2-1,_div.cols()-_div.cols()+d2)).cwiseProduct(_div.block(d2+1,_div.cols()-d2,_div.rows()-d2-d2-1,_div.cols()-_div.cols()+d2));
//	for(int row = d2+1; row < _div.rows-d2; ++row) {
//		const float* div_ptr = _div.ptr<float>(row);
//		const float* ii_ptr1 = _integral_image.ptr<float>(row+d2);
//		const float* ii_ptr2 = _integral_image.ptr<float>(row-d2-1);
//		float* ai_ptr = average_image.ptr<float>(row);
//		for(int col = 0; col < d2+1; ++col) {
//			int col_idx = col+d2;
//			ai_ptr[col] = (ii_ptr1[col_idx]-ii_ptr2[col_idx])*div_ptr[col];
//		}
//		for(int col = d2+1; col < _div.cols-d2; ++col) {
//			int col_idx1 = col+d2;
//			int col_idx2 = col-d2-1;
//			ai_ptr[col] = (ii_ptr1[col_idx1]-ii_ptr1[col_idx2] - ii_ptr2[col_idx1]+ii_ptr2[col_idx2])*a;
//		}
//		for(int col = _div.cols-d2; col < _div.cols; ++col) {
//			int col_idx1 = _div.cols-1;
//			int col_idx2 = col-d2-1;
//			ai_ptr[col] = (ii_ptr1[col_idx1]-ii_ptr1[col_idx2] - ii_ptr2[col_idx1]+ii_ptr2[col_idx2])*div_ptr[col];
//		}
//	}

	average_image.block(_div.rows()-d2,0,_div.rows()-_div.rows()+d2,d2+1) =
			(_integral_image.block(_div.rows()-1,d2,1,d2+1).replicate(_div.rows()-_div.rows()+d2,1)
					-_integral_image.block(_div.rows()-d2-d2-1,d2,_div.rows()-_div.rows()+d2,d2+1)).cwiseProduct(_div.block(_div.rows()-d2,0,_div.rows()-_div.rows()+d2,d2+1));
	average_image.block(_div.rows()-d2,d2+1,_div.rows()-_div.rows()+d2,_div.cols()-d2-d2-1) =
			(_integral_image.block(_div.rows()-1,d2+1+d2,1,_div.cols()-d2-d2-1).replicate(_div.rows()-_div.rows()+d2,1)
					-_integral_image.block(_div.rows()-d2-d2-1,d2+1-d2-1,_div.rows()-_div.rows()+d2,_div.cols()-d2-d2-1)
					-_integral_image.block(_div.rows()-1,d2+1-d2-1,1,_div.cols()-d2-d2-1).replicate(_div.rows()-_div.rows()+d2,1)
					+_integral_image.block(_div.rows()-d2-d2-1,d2+1-d2-1,_div.rows()-_div.rows()+d2,_div.cols()-d2-d2-1)).cwiseProduct(_div.block(_div.rows()-d2,d2+1,_div.rows()-_div.rows()+d2,_div.cols()-d2-d2-1));
	average_image.block(_div.rows()-d2,_div.cols()-d2,_div.rows()-_div.rows()+d2,_div.cols()-_div.cols()+d2) =
			(_integral_image.block(_div.rows()-1,_div.cols()-d2,1,1).replicate(_div.rows()-_div.rows()+d2,_div.cols()-_div.cols()+d2)
					-_integral_image.block(_div.rows()-d2-d2-1,_div.cols()-1,_div.rows()-_div.rows()+d2,1).replicate(1,_div.cols()-_div.cols()+d2)
					-_integral_image.block(_div.rows()-1,_div.cols()-d2-d2-1,1,_div.cols()-_div.cols()+d2).replicate(_div.rows()-_div.rows()+d2,1)
					+_integral_image.block(_div.rows()-d2-d2-1,_div.cols()-d2-d2-1,_div.rows()-_div.rows()+d2,_div.cols()-_div.cols()+d2)).cwiseProduct(_div.block(_div.rows()-d2,_div.cols()-d2,_div.rows()-_div.rows()+d2,_div.cols()-_div.cols()+d2));

//	for(int row = _div.rows-d2; row < _div.rows; ++row) {
//		const float* div_ptr = _div.ptr<float>(row);
//		const float* ii_ptr1 = _integral_image.ptr<float>(_div.rows-1);
//		const float* ii_ptr2 = _integral_image.ptr<float>(row-d2-1);
//		float* ai_ptr = average_image.ptr<float>(row);
//		for(int col = 0; col < d2+1; ++col) {
//			int col_idx = col+d2;
//			ai_ptr[col] = (ii_ptr1[col_idx]-ii_ptr2[col_idx])*div_ptr[col];
//		}
//		for(int col = d2+1; col < _div.cols-d2; ++col) {
//			int col_idx1 = col+d2;
//			int col_idx2 = col-d2-1;
//			ai_ptr[col] = (ii_ptr1[col_idx1]-ii_ptr2[col_idx1] - ii_ptr1[col_idx2]+ii_ptr2[col_idx2])*div_ptr[col];
//		}
//		for(int col = _div.cols-d2; col < _div.cols; ++col) {
//			int col_idx1 = _div.cols-1;
//			int col_idx2 = col-d2-1;
//			ai_ptr[col] = (ii_ptr1[col_idx1]-ii_ptr2[col_idx1] - ii_ptr1[col_idx2]+ii_ptr2[col_idx2])*div_ptr[col];
//		}
//	}
	return average_image;
}

void FastGaussian::precomputeDivisors(int _d, cv::Mat& _div) {
	int d2 = _d/2;
	float a = (float)_d*_d;
	for(int row = 0; row < d2+1; ++row) {
		for(int col = 0; col < d2+1; ++col) {
			_div.at<float>(row,col) = (col+d2+1)*(row+d2+1);
		}
		for(int col = d2+1; col < _div.cols-d2; ++col) {
			_div.at<float>(row,col) = (float)_d*(row+d2+1);
		}
		for(int col = _div.cols-d2; col < _div.cols; ++col) {
			_div.at<float>(row,col) = (_div.cols-col+d2)*(row+d2+1);
		}
	}
	for(int row = d2+1; row < _div.rows-d2; ++row) {
		for(int col = 0; col < d2+1; ++col) {
			_div.at<float>(row,col) = (col+d2+1)*(float)_d;
		}
		for(int col = d2+1; col < _div.cols-d2; ++col) {
			_div.at<float>(row,col) = a;
		}
		for(int col = _div.cols-d2; col < _div.cols; ++col) {
			_div.at<float>(row,col) = (_div.cols-col+d2)*(float)_d;
		}
	}
	for(int row = _div.rows-d2; row < _div.rows; ++row) {
		for(int col = 0; col < d2+1; ++col) {
			_div.at<float>(row,col) = (_div.rows-row+d2)*(col+d2+1);
		}
		for(int col = d2+1; col < _div.cols-d2; ++col) {
			_div.at<float>(row,col) = (_div.rows-row+d2)*(float)_d;
		}
		for(int col = _div.cols-d2; col < _div.cols; ++col) {
			_div.at<float>(row,col) = (_div.rows-row+d2)*(_div.cols-col+d2);
		}
	}
	for(int row = 0; row < _div.rows; ++row) {
		for(int col = 0; col < _div.cols; ++col) {
			_div.at<float>(row,col) = 1.0/_div.at<float>(row,col);
		}
	}
}

void FastGaussian::precomputeDivisors(int _d, Eigen::MatrixXf& _div) {
	int d2 = _d/2;
	float a = (float)_d*_d;
	for(int row = 0; row < d2+1; ++row) {
		for(int col = 0; col < d2+1; ++col) {
			_div(row,col) = (col+d2+1)*(row+d2+1);
		}
		for(int col = d2+1; col < _div.cols()-d2; ++col) {
			_div(row,col) = (float)_d*(row+d2+1);
		}
		for(int col = _div.cols()-d2; col < _div.cols(); ++col) {
			_div(row,col) = (_div.cols()-col+d2)*(row+d2+1);
		}
	}
	for(int row = d2+1; row < _div.rows()-d2; ++row) {
		for(int col = 0; col < d2+1; ++col) {
			_div(row,col) = (col+d2+1)*(float)_d;
		}
		for(int col = d2+1; col < _div.cols()-d2; ++col) {
			_div(row,col) = a;
		}
		for(int col = _div.cols()-d2; col < _div.cols(); ++col) {
			_div(row,col) = (_div.cols()-col+d2)*(float)_d;
		}
	}
	for(int row = _div.rows()-d2; row < _div.rows(); ++row) {
		for(int col = 0; col < d2+1; ++col) {
			_div(row,col) = (_div.rows()-row+d2)*(col+d2+1);
		}
		for(int col = d2+1; col < _div.cols()-d2; ++col) {
			_div(row,col) = (_div.rows()-row+d2)*(float)_d;
		}
		for(int col = _div.cols()-d2; col < _div.cols(); ++col) {
			_div(row,col) = (_div.rows()-row+d2)*(_div.cols()-col+d2);
		}
	}
	for(int row = 0; row < _div.rows(); ++row) {
		for(int col = 0; col < _div.cols(); ++col) {
			_div(row,col) = 1.0/_div(row,col);
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

EigenMat FastGaussian::smooth(EigenMat&& _in) {
	EigenMat out;
	EigenMat integral_image = createIntegralImage(_in);
	for(int i = 0; i < n_-1; ++i) {
		out = average(integral_image,widths_[i],divisors_e_[i]);
		integral_image = createIntegralImage(out);
	}
	out = average(integral_image,widths_[n_-1],divisors_e_[n_-1]);
	return out;
}

ScaleSpace::ScaleSpace() :
		camera_(Camera()),
		filter_{rebvio::FastGaussian(camera_,3.56359,3),rebvio::FastGaussian(camera_,filter_[0].sigma_true_*1.2599,3)}
{
		dog_ = cv::Mat(camera_.rows_,camera_.cols_,CV_32FC1,cv::Scalar(0));
		gradient_mag_ = cv::Mat(camera_.rows_,camera_.cols_,CV_32FC1,cv::Scalar(0));
}

ScaleSpace::~ScaleSpace() {
}

void ScaleSpace::build(cv::Mat& _image) {
	REBVIO_TIMER_TICK();
//	REBVIO_NAMED_TIMER_TICK(OpenCV);
	scale_[0] = filter_[0].smooth(_image);
	scale_[1] = filter_[1].smooth(_image);
//	REBVIO_NAMED_TIMER_TOCK(OpenCV);

//	Eigen::Map<EigenMat> img(_image.ptr<float>(),_image.rows,_image.cols);
//	REBVIO_NAMED_TIMER_TICK(Eigen);
//	scale_e_[0] = filter_[0].smooth(img);
//	scale_e_[1] = filter_[1].smooth(img);
//	REBVIO_NAMED_TIMER_TOCK(Eigen);
	calculateDoG();
	calculateGradientMagnitude();
	REBVIO_TIMER_TOCK();
}

void ScaleSpace::calculateDoG() {
	for(int row = 0; row < dog_.rows; ++row) {
		float* dog_ptr = dog_.ptr<float>(row);
		const float* i0_ptr  = scale_[0].ptr<float>(row);
		const float* i1_ptr  = scale_[1].ptr<float>(row);
		for(int col = 0; col < dog_.cols; ++col) {
			dog_ptr[col] = i1_ptr[col]-i0_ptr[col]; // LoG is approximated with the difference of the larger sigma s1 and the smaller sigma s0: s1-s0
		}
	}
}

void ScaleSpace::calculateGradientMagnitude() {
	for(int row = 1; row < gradient_mag_.rows-1; ++row) {
		const float* ic_ptr = scale_[0].ptr<float>(row);
		const float* irl_ptr = scale_[0].ptr<float>(row-1);
		const float* iru_ptr = scale_[0].ptr<float>(row+1);
		float* mag_ptr = gradient_mag_.ptr<float>(row);
		for(int col = 1; col < gradient_mag_.cols-1; ++col) {
			float dx = ic_ptr[col+1]-ic_ptr[col-1];
			float dy = iru_ptr[col]-irl_ptr[col];
			mag_ptr[col] = dx*dx+dy*dy; // squared magnitude of the image gradient in the first scale
		}
	}
}

} /* namespace rebvio */
