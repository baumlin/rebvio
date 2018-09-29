/*
 * undistorter.hpp
 *
 *  Created on: Sep 26, 2018
 *      Author: baumlin
 */

#ifndef INCLUDE_REBVIO_UNDISTORTER_HPP_
#define INCLUDE_REBVIO_UNDISTORTER_HPP_

#include "rebvio/camera.hpp"
#include "opencv2/core.hpp"

namespace rebvio {

class Undistorter {
public:
	struct Field {
		int num;
		int idx[4];
		types::Float w[4];
		int iw[4];
		int nn_idx;
	};
public:
	class Map {
	public:
		Map(rebvio::Camera::SharedPtr _camera) : rows_(_camera->rows_), cols_(_camera->cols_) {
			map_ = new Undistorter::Field*[rows_];
			for(int row = 0; row < rows_; ++row) {
				map_[row] = new Undistorter::Field[cols_];
			}
		}

		~Map() {
			for(int row = 0; row < rows_; ++row) {
				delete[] map_[row];
			}
			delete[] map_;
		}

		Undistorter::Field** get() {
			return map_;
		}

	private:
		Undistorter::Field** map_;
		unsigned int rows_;
		unsigned int cols_;
	};

public:
	Undistorter(rebvio::Camera::SharedPtr _camera);
	~Undistorter();
	cv::Mat undistort(cv::Mat _in);

private:
	inline types::Float bilinearInterpolation(cv::Mat _in, int _row, int _col);
	void distort(types::Float& _x, types::Float& _y);
	inline bool isValid(int _row, int _col) {
		return (_row >=0 && _col >= 0 && _row < camera_->rows_ && _col < camera_->cols_);
	}
	inline int getIndex(types::Float _row, types::Float _col) {
		int row = std::round(_row);
		int col = std::round(_col);
		if(row >= camera_->rows_ || row < 0 || col >= camera_->cols_ || col < 0) return -1;
		return row*camera_->cols_+col;
	}

private:
	static constexpr int i_shift = 16;
	static constexpr types::Float i_mult = (1<<i_shift);

private:
	rebvio::Camera::SharedPtr camera_;
	rebvio::Undistorter::Map map_;

};

} /* namespace rebvio */



#endif /* INCLUDE_REBVIO_UNDISTORTER_HPP_ */
