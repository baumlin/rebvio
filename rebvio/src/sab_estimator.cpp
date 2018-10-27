/*
 * sab_estimator.cpp
 *
 *  Created on: Oct 27, 2018
 *      Author: baumlin
 */

#include "rebvio/sab_estimator.hpp"
#include "TooN/SVD.h"
#include "TooN/so3.h"
#include "TooN/Cholesky.h"

namespace rebvio {


SABEstimator::SABEstimator(SABEstimator::Config& _config) :
		config_(_config) {}

SABEstimator::~SABEstimator() {}

int SABEstimator::gaussNewton(rebvio::types::Vector7f& _X, int _iter_max, types::Float _a_tol, types::Float _r_tol) {

	// Solving the problem JtJ*h = -JtF for h (correction applied to the state _X to get it closer to solution)
	types::Vector7f h;
	types::Matrix7f JtJ; // Product of transposed jacobian with itself: J^T*J
	types::Vector7f JtF; // Product of transposed jacobian with the residual vector F = f(_X), where f(_X) is the nonlinear error function

	int i = 0;
	for(; i < _iter_max; ++i) {
		problem(JtJ,JtF,_X);
		TooN::SVD<7> solution(JtJ);
		h = solution.backsub(-JtF);  // compute h = JtJ^(-1) * -JtF
		_X += h;
		_X = TooN::makeVector(std::atan2(std::sin(_X[0]),std::cos(_X[0])),_X[1],_X[2],_X[3],saturate(_X[4],5e-1/25),saturate(_X[5],5e-1/25),saturate(_X[6],5e-1/25));
		if(TooN::norm(h) < _a_tol || TooN::norm(h)/(TooN::norm(_X)+1e-20) < _r_tol) break;
	}
	return i;
}

bool SABEstimator::problem(rebvio::types::Matrix7f& _JtJ, rebvio::types::Vector7f& _JtF, const rebvio::types::Vector7f& _X) {

	// Minimizing E_corr (Equation (40) in "Realtime Edge Based Visual Inertial Odometry for MAV
	// Teleoperation in Indoor Environments" (Tarrio & Pedre, 2017)
	types::Float a = _X[0];
	rebvio::types::Vector3f g = _X.slice<1,3>();
	rebvio::types::Vector3f b = _X.slice<4,3>();
	rebvio::types::Vector3f& a_s = config_.a_s;
	rebvio::types::Vector3f& a_v = config_.a_v;

	// Nonlinear error function vector (augmented with the gravity-corrected acceleration in polar coordinates and the squared norm of standard gravity)
	// F = [r; g; alpha; gravity; visual rotation bias]
	rebvio::types::Vector11f F = TooN::Zeros;
	F.slice<0,3>() = (a_s+g)*std::cos(a)-a_v*std::sin(a);  // Error function gravity-corrected acceleration in polar coordinates
	F[3]=g*g-config_.G*config_.G;                          // Error function squared norm of standard gravity
	F[4] = _X[0]-config_.x_p[0];                           // Error function angle
	if(F[4] > M_PI)                                        // Error correction on the angle (due to cyclicity)
		F[4] -= 2.0*M_PI;
	else if(F[4] < -M_PI)
		F[4] += 2.0*M_PI;
	TooN::SO3<types::Float> Rb(b);
	F.slice<5,3>() = Rb*g-config_.x_p.slice<1,3>();        // Error function gravity
	F.slice<8,3>() = b-config_.x_p.slice<4,3>();           // Error function visual rotation bias

	// Derivative of F w.r.t. a
	types::Vector11f dFda = TooN::Zeros;
	dFda.slice<0,3>() = -(a_s+g)*std::sin(a)-a_v*std::cos(a);
	dFda[4] = 1.0;

	// Derivative of F w.r.t. gravity and visual rotation bias (x1 = [gravity vector, visual rotation bias vector]
	// dFdx1 = | x 0 0 0 0 0 |
	//         | 0 x 0 0 0 0 |
	//         | 0 0 x 0 0 0 |
	//         | x x x 0 0 0 |
	//         | 0 0 0 0 0 0 |
	//         | x x x x x x |
	//         | x x x x x x |
	//         | x x x x x x |
	//         | 0 0 0 1 0 0 |
	//         | 0 0 0 0 1 0 |
	//         | 0 0 0 0 0 1 |
	TooN::Matrix<11,6,types::Float> dFdx1 = TooN::Zeros;
	types::Vector3f Rg = Rb*g;
	types::Matrix3f Gx = TooN::Data( 0.0  , Rg[2], -Rg[1],
																	-Rg[2],  0.0 ,  Rg[0],
																	 Rg[1],-Rg[0],   0.0  );
	dFdx1.slice<0,0,3,3>() = TooN::Identity*std::cos(a);
	dFdx1.slice<3,0,1,3>() = 2.0*g.as_row();
	dFdx1.slice<5,0,3,3>() = Rb.get_matrix();
	dFdx1.slice<5,3,3,3>() = Gx;
	dFdx1.slice<8,3,3,3>() = TooN::Identity;

	// Calculate the covariance matrix P
	// P = | x x x 0 0 0 0 0 0 0 0 |
	//     | x x x 0 0 0 0 0 0 0 0 |
	//     | x x x 0 0 0 0 0 0 0 0 |
	//     | 0 0 0 x 0 0 0 0 0 0 0 |
	//     | 0 0 0 0 x x x x x x x |
	//     | 0 0 0 0 x x x x x x x |
	//     | 0 0 0 0 x x x x x x x |
	//     | 0 0 0 0 x x x x x x x |
	//     | 0 0 0 0 x x x x x x x |
	//     | 0 0 0 0 x x x x x x x |
	//     | 0 0 0 0 x x x x x x x |
	types::Matrix11f P = TooN::Zeros;
	types::Matrix3f Pz = std::sin(a)*std::sin(a)*config_.Rv+std::cos(a)*std::cos(a)*config_.Rs;
	P.slice<0,0,3,3>() = Pz;
	P(3,3) = config_.Rg;
	P.slice<4,4,7,7>() = config_.Pp;

	// W is the inverse of the covariance matrix P for the weighted least-squares problem:
	// W = | x x x 0 0 0 0 0 0 0 0 |
	//     | x x x 0 0 0 0 0 0 0 0 |
	//     | x x x 0 0 0 0 0 0 0 0 |
	//     | 0 0 0 x 0 0 0 0 0 0 0 |
	//     | 0 0 0 0 x x x x x x x |
	//     | 0 0 0 0 x x x x x x x |
	//     | 0 0 0 0 x x x x x x x |
	//     | 0 0 0 0 x x x x x x x |
	//     | 0 0 0 0 x x x x x x x |
	//     | 0 0 0 0 x x x x x x x |
	//     | 0 0 0 0 x x x x x x x |
	types::Matrix11f W = TooN::Zeros;
	W.slice<0,0,3,3>() = TooN::Cholesky<3,types::Float>(Pz).get_inverse();
	W(3,3) = 1.0/config_.Rg;
	W.slice<4,4,7,7>() = TooN::Cholesky<7,types::Float>(config_.Pp).get_inverse();

	// Derivative of P w.r.t. a:
	// dPda = | x x x 0 0 0 0 0 0 0 |
	//        | x x x 0 0 0 0 0 0 0 |
	//        | x x x 0 0 0 0 0 0 0 |
	//        | 0 0 0 0 0 0 0 0 0 0 |
	//        | 0 0 0 0 0 0 0 0 0 0 |
	//        | 0 0 0 0 0 0 0 0 0 0 |
	//        | 0 0 0 0 0 0 0 0 0 0 |
	//        | 0 0 0 0 0 0 0 0 0 0 |
	//        | 0 0 0 0 0 0 0 0 0 0 |
	//        | 0 0 0 0 0 0 0 0 0 0 |
	//        | 0 0 0 0 0 0 0 0 0 0 |
	types::Matrix11f dPda = TooN::Zeros;
	dPda.slice<0,0,3,3>() = 2.0*std::sin(a)*std::cos(a)*(config_.Rv-config_.Rs);

	// Derivative of W w.r.t. a
	// dWda = | x x x 0 0 0 0 0 0 0 |
	//        | x x x 0 0 0 0 0 0 0 |
	//        | x x x 0 0 0 0 0 0 0 |
	//        | 0 0 0 0 0 0 0 0 0 0 |
	//        | 0 0 0 0 0 0 0 0 0 0 |
	//        | 0 0 0 0 0 0 0 0 0 0 |
	//        | 0 0 0 0 0 0 0 0 0 0 |
	//        | 0 0 0 0 0 0 0 0 0 0 |
	//        | 0 0 0 0 0 0 0 0 0 0 |
	//        | 0 0 0 0 0 0 0 0 0 0 |
	//        | 0 0 0 0 0 0 0 0 0 0 |
	types::Matrix11f dWda = -W*dPda*W;

	_JtJ(0,0) = 0.25*F*dWda*P*dWda*F+dFda*dWda*F+dFda*W*dFda;
	_JtJ.slice<1,0,6,1>() = (0.5*dFdx1.T()*dWda*F+dFdx1.T()*W*dFda).as_col();
	_JtJ.slice<0,1,1,6>() = _JtJ.slice<1,0,6,1>().T();
	_JtJ.slice<1,1,6,6>() = dFdx1.T()*W*dFdx1;

	_JtF[0] = 0.5*F*dWda*F+dFda*W*F;
	_JtF.slice<1,6>() = dFdx1.T()*W*F;

	return true;
}

types::Float SABEstimator::saturate(types::Float _t, types::Float _limit) {
	return (_t > _limit) ? _limit : ((_t < -_limit) ? -_limit : _t);
}

}
