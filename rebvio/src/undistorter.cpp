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
			types::Float qdx = types::Float(col)-camera_->cx_;
			types::Float qdy = types::Float(row)-camera_->cy_;
			distort(qdx,qdy);
			types::Float pdx = qdx+camera_->cx_;
			types::Float pdy = qdy+camera_->cy_;
			p.nn_idx = getIndex(pdy,pdx);

			types::Float p00x = std::floor(pdx);
			types::Float p00y = std::floor(pdy);
			types::Float p01x = std::floor(pdx)+1;
			types::Float p01y = std::floor(pdy);
			types::Float p10x = std::floor(pdx);
			types::Float p10y = std::floor(pdy)+1;
			types::Float p11x = std::floor(pdx)+1;
			types::Float p11y = std::floor(pdy)+1;

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
				types::Float sum_w = 0.0;
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

void Undistorter::distort(types::Float& _x, types::Float& _y) {
	types::Float xp = _x/camera_->fm_;
	types::Float yp = _y/camera_->fm_;
	types::Float r2 = xp*xp+yp*yp;
	types::Float xpp = xp*(1.0+r2*(camera_->k1_+r2*(camera_->k2_+r2*camera_->k3_)))+2.0*camera_->p1_*xp*yp+camera_->p2_*(r2+2.0*xp*xp);
	types::Float ypp = yp*(1.0+r2*(camera_->k1_+r2*(camera_->k2_+r2*camera_->k3_)))+camera_->p1_*(r2+2.0*yp*yp)+2.0*camera_->p2_*xp*yp;

	_x = xpp*camera_->fx_;
	_y = ypp*camera_->fy_;
}

cv::Mat Undistorter::undistort(cv::Mat _in) {
	cv::Mat out = cv::Mat(camera_->rows_,camera_->cols_,CV_FLOAT_PRECISION);
	for(int row = 0; row < camera_->rows_; ++row) {
		types::Float* out_ptr = out.ptr<types::Float>(row);
		for(int col = 0; col < camera_->cols_; ++col) {
			out_ptr[col] = bilinearInterpolation(_in,row,col);
		}
	}
	return out;
}

types::Float Undistorter::bilinearInterpolation(cv::Mat _in, int _row, int _col) {
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

	return types::Float(uint8_t(r)+uint8_t(g)+uint8_t(b));
}



} /* namespace rebvio */



