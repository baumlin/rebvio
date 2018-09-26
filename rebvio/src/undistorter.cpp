/*
 * undistorter.cpp
 *
 *  Created on: Sep 26, 2018
 *      Author: baumlin
 */

#include "rebvio/undistorter.hpp"

namespace rebvio {

Undistorter::Undistorter(rebvio::Camera::SharedPtr _camera) :
	camera_(_camera),
	map_(_camera)
{
	for(int row = 0; row < camera_->rows_; ++row) {
		for(int col = 0; col < camera_->cols_; ++col) {
			Undistorter::Field& p = map_.get()[row][col];
			float qdx = float(col)-camera_->cx_;
			float qdy = float(row)-camera_->cy_;
			distort(qdx,qdy);
			float pdx = qdx+camera_->cx_;
			float pdy = qdy+camera_->cy_;
			p.nn_idx = getIndex(pdy,pdx);

			float p00x = std::floor(pdx);
			float p00y = std::floor(pdy);
			float p01x = std::floor(pdx)+1;
			float p01y = std::floor(pdy);
			float p10x = std::floor(pdx);
			float p10y = std::floor(pdy)+1;
			float p11x = std::floor(pdx)+1;
			float p11y = std::floor(pdy)+1;

			p.num = 0;
			if(isValid(p00y,p00x)) {
				p.w[p.num] = (p11x-pdx)*(p11y-pdy);
				p.idx[p.num] = getIndex(p00y,p00x);
				++p.num;
			}
			if(isValid(p01y,p01x)) {
				p.w[p.num] = (pdx-p00x)*(p11y-pdy);
				p.idx[p.num] = getIndex(p01y,p01x);
				++p.num;
			}
			if(isValid(p10y,p10x)) {
				p.w[p.num] = (p11x-pdx)*(pdy-p00y);
				p.idx[p.num] = getIndex(p10y,p10x);
				++p.num;
			}
			if(isValid(p11y,p11x)) {
				p.w[p.num] = (pdx-p00x)*(pdy-p00y);
				p.idx[p.num] = getIndex(p11y,p11x);
				++p.num;
			}
			if(p.num > 0) {
				float sum_w = 0.0;
				for(int i = 0; i < p.num; ++i) {
					sum_w += p.w[i];
				}
				for(int i = 0; i < p.num; ++i) {
					p.w[i] /= sum_w;
					p.iw[i] = p.w[i]*i_mult;
				}

			}
		}
	}
}

Undistorter::~Undistorter() {}

void Undistorter::distort(float& _x, float& _y) {
	float xp = _x/camera_->fm_;
	float yp = _y/camera_->fm_;
	float r2 = xp*xp+yp*yp;
	float xpp = xp*(1.0+r2*(camera_->k1_+r2*(camera_->k2_+r2*camera_->k3_)))+2.0*camera_->p1_*xp*yp+camera_->p2_*(r2+2.0*xp*xp);
	float ypp = yp*(1.0+r2*(camera_->k1_+r2*(camera_->k2_+r2*camera_->k3_)))+camera_->p1_*(r2+2.0*yp*yp)+2.0*camera_->p2_*xp*yp;

	_x = xpp*camera_->fx_;
	_y = ypp*camera_->fy_;
}

cv::Mat Undistorter::undistort(cv::Mat _in) {
	cv::Mat out = cv::Mat(camera_->rows_,camera_->cols_,CV_32FC1);
	for(int row = 0; row < camera_->rows_; ++row) {
		float* out_ptr = out.ptr<float>(row);
		for(int col = 0; col < camera_->cols_; ++col) {
			out_ptr[col] = bilinearInterpolation(_in,row,col);
		}
	}
	return out;
}

float Undistorter::bilinearInterpolation(cv::Mat _in, int _row, int _col) {
	int r = 0;
	int g = 0;
	int b = 0;
	Undistorter::Field& p = map_.get()[_row][_col];
	for(int i = 0; i < p.num; ++i) {
		r += p.iw[i]*_in.at<cv::Vec3b>(p.idx[i])[0];
		g += p.iw[i]*_in.at<cv::Vec3b>(p.idx[i])[1];
		b += p.iw[i]*_in.at<cv::Vec3b>(p.idx[i])[2];
	}

	r >>= i_shift;
	g >>= i_shift;
	b >>= i_shift;

	return float(uint8_t(r)+uint8_t(g)+uint8_t(b));
}



} /* namespace rebvio */



