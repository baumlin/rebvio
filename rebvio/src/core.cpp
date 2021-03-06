/*
 * edge_tracker.cpp
 *
 *  Created on: Sep 2, 2018
 *      Author: baumlin
 */

#include <rebvio/core.hpp>
#include "rebvio/sab_estimator.hpp"
#include "rebvio/util/timer.hpp"
#include <TooN/SVD.h>
#include <TooN/Cholesky.h>
#include <TooN/helpers.h>
#include <cmath>

namespace rebvio {

Core::Core(rebvio::Camera::SharedPtr _camera, rebvio::CoreConfig::SharedPtr _config) :
	config_(_config),
	camera_(_camera),
	distance_field_(camera_->rows_,camera_->cols_,config_->search_range),
	frame_count_(0)
{

}

Core::~Core() {
	// TODO Auto-generated destructor stub
}

CoreConfig::SharedPtr Core::config() { return config_; }

void Core::buildDistanceField(rebvio::EdgeMap::SharedPtr _map) {
	REBVIO_TIMER_TICK();
	distance_field_.build(_map);
	REBVIO_TIMER_TOCK();
}

bool Core::testfk(const rebvio::types::KeyLine& _keyline1, const rebvio::types::KeyLine& _keyline2, const types::Float& _similarity_threshold) {
	types::Float norm_squared = _keyline2.gradient_norm*_keyline2.gradient_norm;
	types::Float dot_product = _keyline1.gradient[0]*_keyline2.gradient[0]+_keyline1.gradient[1]*_keyline2.gradient[1];
	if(std::fabs(dot_product-norm_squared) > _similarity_threshold*norm_squared)	return false; // |g1*g2 - g2*g2|/(g2*g2) > threshold ?
	return true;
}

types::Float Core::calculatefJ(rebvio::EdgeMap::SharedPtr _map, int _f_inx, types::Float& _df_dx, types::Float& _df_dy, rebvio::types::KeyLine& _keyline,
		const types::Float& _px, const types::Float& _py, int& _mnum, types::Float& _fi) {

	if(distance_field_[_f_inx].id < 0) {
		_df_dx = 0.0;
		_df_dy = 0.0;
		return config_->search_range/_keyline.sigma_rho;
	}

	const types::KeyLine& keyline = (*(distance_field_.map()))[distance_field_[_f_inx].id];
	if(!Core::testfk(keyline,_keyline,config_->match_treshold)) {
		_df_dx = 0.0;
		_df_dy = 0.0;
		return config_->search_range/_keyline.sigma_rho;
	}

	types::Float dx = _px-keyline.pos[0];
	types::Float dy = _py-keyline.pos[1];

	types::Float gnx = keyline.gradient[0]/keyline.gradient_norm;
	types::Float gny = keyline.gradient[1]/keyline.gradient_norm;
	_fi = (dx*gnx+dy*gny); // Residual in direction of the gradient

	_df_dx = gnx/_keyline.sigma_rho;
	_df_dy = gny/_keyline.sigma_rho;

	++_mnum;
	_keyline.match_id_forward = distance_field_[_f_inx].id;

	return _fi/_keyline.sigma_rho;
}

types::Float Core::tryVel(rebvio::EdgeMap::SharedPtr _map, rebvio::types::Matrix3f& _JtJ, rebvio::types::Vector3f& _JtF, const rebvio::types::Vector3f& _vel,
						 types::Float _sigma_rho_min, types::Float* _residuals) {
	types::Float score = 0.0;
	_JtJ = TooN::Zeros;
	_JtF = TooN::Zeros;
	int mnum = 0;
	for(int idx = 0; idx < _map->size(); ++idx) {
		rebvio::types::KeyLine& keyline = (*_map)[idx];

		keyline.match_id_forward = -1; // TODO: is this necessary?
		if(_map->threshold() > 0.0 && keyline.gradient_norm < _map->threshold()) continue;

		// Use keyline if uncertainty is not too high or it has certain number of matches
		if(keyline.sigma_rho > _sigma_rho_min || keyline.matches < std::min(config_->min_match_threshold,frame_count_)) continue;

		types::Float weight = 1.0;
		if(_residuals[idx] > config_->reweight_distance) weight = config_->reweight_distance/_residuals[idx];

		types::Float z_p = 1.0/keyline.rho+_vel[2];
		types::Float f;
		if(z_p <= 0.0) {
			f = (1.0/keyline.sigma_rho)*config_->search_range*weight;
			score += f*f;
			continue;
		}

		types::Float rho_p = 1.0/z_p;
		types::Float p_x = rho_p*(_vel[0]*camera_->fm_-_vel[2]*keyline.pos_img[0])+keyline.pos_img[0];
		types::Float p_y = rho_p*(_vel[1]*camera_->fm_-_vel[2]*keyline.pos_img[1])+keyline.pos_img[1];

		types::Float p_xc = p_x + camera_->cx_;
		types::Float p_yc = p_y + camera_->cy_;

		int x = static_cast<int>(p_xc+0.5);
		int y = static_cast<int>(p_yc+0.5);

		if(x < 1 || y < 1 || x >= camera_->cols_-1 || y >= camera_->rows_-1) {
			f = (1.0/keyline.sigma_rho)*config_->search_range*weight;
			score += f*f;
			continue;
		}

		types::Float df_dx;
		types::Float df_dy;
		types::Float fi;
		f = calculatefJ(_map,y*camera_->cols_+x,df_dx,df_dy,keyline,p_xc,p_yc,mnum,fi);
		f *= weight;
		score += f*f;
		types::Float jx = rho_p*camera_->fm_*df_dx*weight;
		types::Float jy = rho_p*camera_->fm_*df_dy*weight;
		types::Float jz = -rho_p*(p_x*df_dx+p_y*df_dy)*weight;

		_JtJ(0,0) += jx*jx;
		_JtJ(1,1) += jy*jy;
		_JtJ(2,2) += jz*jz;
		_JtJ(0,1) += jx*jy;
		_JtJ(0,2) += jx*jz;
		_JtJ(1,2) += jy*jz;

		_JtF[0] += jx*f;
		_JtF[1] += jy*f;
		_JtF[2] += jz*f;

		_residuals[idx] = std::fabs(fi);
	}
	_JtJ(1,0) = _JtJ(0,1);
	_JtJ(2,0) = _JtJ(0,2);
	_JtJ(2,1) = _JtJ(1,2);

	return score;
}

types::Float Core::minimizeVel(rebvio::EdgeMap::SharedPtr _map, rebvio::types::Vector3f& _vel, rebvio::types::Matrix3f& _Rvel) {

	REBVIO_TIMER_TICK();
	types::Float sigma_rho_min = _map->estimateQuantile(config_->quantile_cutoff,config_->quantile_num_bins);

	types::Matrix3f JtJ, ApI, JtJnew;
	types::Vector3f JtF, JtFnew;
	types::Vector3f h, Vnew;
	types::Float residuals[_map->size()] = {0.0};
	types::Float F = tryVel(_map,JtJ,JtF,_vel,sigma_rho_min,residuals);

	types::Float v = 2.0;
	types::Float tau = 1e-3;
	types::Float u = tau*TooN::max_element(JtJ).first;
	types::Float gain;

	for(int iter = 0; iter < config_->iterations; ++iter) {
		ApI = JtJ+TooN::Identity*u;
		h = types::invert(ApI)*(-JtF); // Solve ApI*h = -g
		Vnew = _vel+h;
		types::Float Fnew = tryVel(_map,JtJnew,JtFnew,Vnew,sigma_rho_min,residuals);

		gain = (F-Fnew)/(0.5*h*(u*h-JtF));
		if(gain > 0.0) {
			F = Fnew;
			_vel = Vnew;
			JtJ = JtJnew;
			JtF = JtFnew;
			u *= std::max(0.33,1.0-((2.0*gain-1.0)*(2.0*gain-1.0)*(2.0*gain-1.0)));
			v = 2.0;
		} else {
			u *= v;
			v *= 2.0;
		}
	}
	_Rvel = types::invert(JtJ);

	REBVIO_TIMER_TOCK();
	return F;
}

bool Core::extRotVel(rebvio::EdgeMap::SharedPtr _map, const rebvio::types::Vector3f& _vel, rebvio::types::Matrix6f& _Wx, rebvio::types::Vector6f& _X) {

	REBVIO_TIMER_TICK();
	int nm = 0;
	for(int i = 0; i < distance_field_.map()->size(); ++i) {
		if((*distance_field_.map())[i].match_id >= 0) ++nm;
	}

	TooN::Matrix<TooN::Dynamic,6,types::Float> Phi = TooN::Zeros(nm,6);
	TooN::Vector<TooN::Dynamic,types::Float> Y = TooN::Zeros(nm);
	int j = 0;
	for(int idx = 0; idx < distance_field_.map()->size(); ++idx) {
		const rebvio::types::KeyLine& keyline = (*distance_field_.map())[idx];

		if(keyline.match_id < 0) continue;

		types::Float u_x = keyline.gradient[0]/keyline.gradient_norm;
		types::Float u_y = keyline.gradient[1]/keyline.gradient_norm;

		types::Float rho_t = 1.0/(1.0/keyline.rho+_vel[2]);
		types::Float qt_x = keyline.match_pos_img[0] + rho_t*(_vel[0]*camera_->fm_-_vel[2]*keyline.match_pos_img[0]);
		types::Float qt_y = keyline.match_pos_img[1] + rho_t*(_vel[1]*camera_->fm_-_vel[2]*keyline.match_pos_img[1]);

		types::Float q_x = keyline.pos_img[0];
		types::Float q_y = keyline.pos_img[1];

		Phi(j,0) = u_x*rho_t*camera_->fm_;
		Phi(j,1) = u_y*rho_t*camera_->fm_;
		Phi(j,2) = u_x*(-rho_t*q_x) + u_y*(-rho_t*q_y);
		Phi(j,3) = -u_x*q_x*q_y/camera_->fm_ - u_y*(camera_->fm_+q_y*q_y/camera_->fm_);
		Phi(j,4) = u_y*q_x*q_y/camera_->fm_ + u_x*(camera_->fm_+q_x*q_x/camera_->fm_);
		Phi(j,5) = -u_x*q_y + u_y*q_x;

		Y[j] = u_x*(q_x-qt_x)+u_y*(q_y-qt_y);

		types::Float dqvel = u_x*(_vel[0]*camera_->fm_-_vel[2]*keyline.match_pos_img[0]) + u_y*(_vel[1]*camera_->fm_-_vel[2]*keyline.match_pos_img[1]);
		types::Float s_y = std::sqrt(keyline.sigma_rho*keyline.sigma_rho*dqvel*dqvel+config_->pixel_uncertainty*config_->pixel_uncertainty);

		types::Float weight = 1.0;
		if(std::fabs(Y[j]) > config_->reweight_distance) weight = std::fabs(Y[j])/config_->reweight_distance;

		Phi.slice(j,0,1,6) /= s_y*weight;
		Y[j] /= s_y*weight;
		++j;

	}

	if(j != nm) {
		REBVIO_TIMER_TOCK();
		std::cout<<__FILE__<<":"<<__LINE__<<": j != _map->matches()!\n";
		return false;
	}

	types::Matrix6f JtJ = Phi.T()*Phi;
	types::Vector6f JtF = Phi.T()*Y;

	TooN::SVD<6,6,types::Float> SVDpTp(JtJ);
	_X = SVDpTp.backsub(JtF);
	_Wx = JtJ;

	for(int i = 0; i < _X.SizeParameter; ++i) {
		if(std::isnan(_X[i])) {
			REBVIO_TIMER_TOCK();
			std::cout<<__FILE__<<":"<<__LINE__<<": NaN in _X!\n";
			return false;
		}
	}
	REBVIO_TIMER_TOCK();

	return true;
}


types::Vector3f Core::gyroBiasCorrection(rebvio::types::Vector6f& _X, rebvio::types::Matrix6f& _Wx,
								 	 	 	 	 	 	  rebvio::types::Matrix3f& _Wb, const rebvio::types::Matrix3f& _Rg, const rebvio::types::Matrix3f& _Rb) {

	// Minimizing E_gv (Equation (27) in "Realtime Edge Based Visual Inertial Odometry for MAV
	// Teleoperation in Indoor Environments" (Tarrio & Pedre, 2017)
	types::Vector3f dgbias = TooN::Zeros;
	const types::Matrix3f Wg = types::invert(_Rg); // gyro measurement information matrix
	_Wb = types::invert(types::invert(_Wb)+_Rb); // update gyro bias information matrix with gyro bias measurement covariance
	types::Matrix6f Wxb = _Wx;
	types::Matrix3f iWgWb = types::invert(Wg+_Wb);
	Wxb.slice<3,3,3,3>() += Wg*(TooN::Identity-iWgWb*Wg);
	types::Vector6f X1 = _Wx*_X;
	X1.slice<3,3>() += Wg*iWgWb*_Wb*dgbias; // TODO: what is this good for? (dgbias == zero)
	_X = TooN::Cholesky<6,types::Float>(Wxb).get_inverse()*X1;
	dgbias = iWgWb*(Wg*_X.slice<3,3>()+_Wb*dgbias); // TODO: dgbias == zero anyway?
	_Wb = Wg+_Wb; // update bias information matrix (inverse of the covariance matrix
	_Wx.slice<3,3,3,3>() += Wg;  // update rotation state information matrix
	return dgbias;
}

void Core::estimateLs4Acceleration(const rebvio::types::Vector3f& _vel, rebvio::types::Vector3f& _acc,
														 const rebvio::types::Matrix3f& _R, types::Float _dt) {

	static types::Vector3f V = TooN::Zeros;
	static types::Vector3f V0 = TooN::Zeros;
	static types::Vector3f V1 = TooN::Zeros;
	static types::Vector3f V2 = TooN::Zeros;
	static types::Vector3f V3 = TooN::Zeros;
	static types::Float T[5] = {0.0};
	static types::Float Dt[4] = {0.0};

	V3 = _R.T()*V2;
	V2 = _R.T()*V1;
	V1 = _R.T()*V0;
	V0 = _R.T()*V;
	V = _vel;
	for(int i = 0; i < 3; ++i) {
		Dt[i] = Dt[i+1];
	}
	Dt[3] = _dt;

	T[0] = 0.0;
	types::Float mt = 0.0;
	for(int i = 0; i < 4; ++i){
		T[i+1] = T[i]+Dt[i];
		mt += T[i+1];
	}
	mt /= 5.0;

	types::Float den = 0.0;
	for(int i = 0; i < 5; ++i)
		den += (T[i]-mt)*(T[i]-mt);

	types::Float vm;
	types::Float num = 0.0;
	for(int i = 0; i < 3; ++i){

		vm = (V[i]+V0[i]+V1[i]+V2[i]+V[3])/5.0;

		num = (V[i]-vm)*(T[4]-mt);
		num += (V0[i]-vm)*(T[3]-mt);
		num += (V1[i]-vm)*(T[2]-mt);
		num += (V2[i]-vm)*(T[1]-mt);
		num += (V3[i]-vm)*(T[0]-mt);

		if(den > 0.0)
			_acc[i] = num/den;
	}
}

void Core::estimateMeanAcceleration(const rebvio::types::Vector3f _sacc, rebvio::types::Vector3f& _acc, const rebvio::types::Matrix3f& _R) {
	static types::Vector3f A = TooN::Zeros;
	static types::Vector3f A0 = TooN::Zeros;
	static types::Vector3f A1 = TooN::Zeros;
	static types::Vector3f A2 = TooN::Zeros;

	A2 = _R.T()*A1;
	A1 = _R.T()*A0;
	A0 = _R.T()*A;
	A=TooN::makeVector(_sacc[0],_sacc[1],_sacc[2]);

	_acc = 0.25*(A+A0+A1+A2);
}


types::Float Core::estimateBias(const rebvio::types::Vector3f& _sacc, const rebvio::types::Vector3f& _facc, types::Float _kP, const rebvio::types::Matrix3f _Rot,
																	rebvio::types::Vector7f& _X, rebvio::types::Matrix7f& _P, const rebvio::types::Matrix3f& _Qg, const rebvio::types::Matrix3f& _Qrot,
                                  const rebvio::types::Matrix3f& _Qbias, types::Float _QKp, types::Float _Rg, const rebvio::types::Matrix3f& _Rs,
																	const rebvio::types::Matrix3f& _Rf, rebvio::types::Vector3f& _g_est, rebvio::types::Vector3f& _b_est, const rebvio::types::Matrix6f& _Wvw,
																	rebvio::types::Vector6f& _Xvw, types::Float _g_gravit) {

	types::Matrix7f F = TooN::Zeros;
	F(0,0) = _kP;
	F.slice<1,1,3,3>() = _Rot.T();
	F.slice<4,4,3,3>() = TooN::Identity;

	types::Vector3f Gtmp = _X.slice<1,3>();
	types::Matrix3f GProd = TooN::Data(  0.0  , Gtmp[2], -Gtmp[1],
			-Gtmp[2],  0.0  ,  Gtmp[0],
			Gtmp[1],-Gtmp[0],   0.0   );

	types::Matrix7f Q = TooN::Zeros;
	types::Float tan = std::tan(_X[0]);
	Q(0,0) = _QKp/(1.0+tan*tan);
	Q.slice<1,1,3,3>() = GProd.T()*_Qrot*GProd+_Qg;
	Q.slice<4,4,3,3>() = _Qbias;

	_X = F*_X;
	types::Matrix7f Pp = F*_P*F.T()+Q;


	/*Posterior estimation (non linear)*/
	rebvio::SABEstimator::Config params(_facc,_sacc,_g_gravit,_X,_Rf,_Rs,_Rg,Pp);
	rebvio::SABEstimator sab_estimator(params);
	sab_estimator.gaussNewton(_X,20);

	types::Matrix7f JtJ;
	types::Vector7f JtF;
	sab_estimator.problem(JtJ,JtF,_X);

	_P = TooN::Cholesky<7,types::Float>(JtJ).get_inverse();

	types::Float k = std::tan(_X[0]);

	if(k<0 || std::isnan(k) || std::isinf(k))
		k=0;

	_g_est = _X.slice<1,3>();
	_b_est = _X.slice<4,3>();

	types::Matrix3f WVBias = JtJ.slice<4,4,3,3>();

	types::Matrix6f Wb = TooN::Zeros;
	Wb.slice<3,3,3,3>() = WVBias;

	types::Vector3f wc = _Xvw.slice<3,3>() - _b_est;

	types::Vector6f WXc = TooN::Zeros;
	WXc.slice<3,3>() = WVBias*wc;
	types::Vector6f Xc = TooN::Cholesky<6,types::Float>(Wb+_Wvw).get_inverse()*(_Wvw*_Xvw +  WXc);

	_Xvw = Xc;

	if(TooN::isnan(_Xvw)){

		std::cout<<__FILE__<<":"<<__LINE__<<":\n  k=" << k<<", _X="<<_X<<", x_p="<<params.x_p<<"\n  F="<<F<<"\n  Pp="<<Pp<<"\n  Rs="<<params.Rs<<", Rv="<<params.Rv<<"\n";

	}

	return k;
}


void Core::updateInverseDepth(rebvio::types::Vector3f& _vel) {
	for(int idx = 0; idx < distance_field_.map()->size(); ++idx) {
		types::KeyLine& keyline = (*distance_field_.map())[idx];
		if(keyline.match_id >= 0) updateInverseDepthARLU(keyline,_vel);
	}
}

void Core::updateInverseDepthARLU(rebvio::types::KeyLine& _keyline, rebvio::types::Vector3f& _vel) {
	types::Float qx = _keyline.pos_img[0];
	types::Float qy = _keyline.pos_img[1];
	types::Float q0x = _keyline.match_pos_img[0];
	types::Float q0y = _keyline.match_pos_img[1];
	types::Float v_rho = _keyline.sigma_rho*_keyline.sigma_rho;
	types::Float ux = _keyline.match_gradient[0]/_keyline.match_gradient_norm;
	types::Float uy = _keyline.match_gradient[1]/_keyline.match_gradient_norm;
	types::Float Y = ux*(qx-q0x)+uy*(qy-q0y);
	types::Float H = ux*(_vel[0]*camera_->fm_-_vel[2]*q0x)+uy*(_vel[1]*camera_->fm_-_vel[2]*q0y);
	types::Float rho_p = 1.0/(1.0/_keyline.rho+_vel[2]);
	types::Float F = 1.0/(1.0+_keyline.rho*_vel[2]);
	F *= F;
	types::Float p_p = F*v_rho*F + config_->reshape_q_abs*config_->reshape_q_abs;
	types::Float e = Y-H*rho_p;

	types::Float S = H*p_p*H + config_->pixel_uncertainty*config_->pixel_uncertainty;
	types::Float K = p_p*H*(1.0/S);

	_keyline.rho = rho_p+K*e;
	v_rho = (1.0-K*H)*p_p;
	_keyline.sigma_rho = std::sqrt(v_rho);
	if(_keyline.rho < types::RHO_MIN) {
		_keyline.sigma_rho += types::RHO_MIN-_keyline.rho;
		_keyline.rho = types::RHO_MIN;
	} else if(_keyline.rho > types::RHO_MAX) {
		_keyline.rho = types::RHO_MAX;
	} else if(std::isnan(_keyline.rho) || std::isnan(_keyline.sigma_rho) || std::isinf(_keyline.rho) || std::isinf(_keyline.sigma_rho)) {
		std::cerr<<"ERROR NaN or INF RHO in updateInverseDepthARLU()!\n";
		_keyline.rho = types::RHO_INIT;
		_keyline.sigma_rho = types::RHO_MAX;
	}
}


} /* namespace rebvio */
