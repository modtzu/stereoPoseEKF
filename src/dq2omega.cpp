/*
 * dq2omega.cpp
 *
 *  Created on: Jan 17, 2017
 *      Author: xwong
 */

#include "dq2omega.h"

dq2omega::dq2omega() {
	// TODO Auto-generated constructor stub

}

void dq2omega::convert(arma::mat q, arma::mat dqdt, arma::mat& omega) {

	arma::mat cQ = arma::zeros(3,3);
	cQ(0,1) = -q(2,0);
	cQ(1,0) =  q(2,0);

	cQ(0,2) =  q(1,0);
	cQ(2,0) = -q(1,0);

	cQ(1,2) = -q(0,0);
	cQ(2,1) =  q(0,0);

	arma::mat Bi = 2*(arma::eye(3,3) - cQ)/(1 + arma::dot(q.col(0),q.col(0)));

	omega = Bi*dqdt;

}

void dq2omega::compute_cov_omega(arma::mat q, arma::mat dqdt, arma::mat omega, arma::mat cov_q,
		arma::mat cov_dq, arma::mat& cov_omega) {

	///////////////////////////////////////////////
	/* Omega = B^(-1) dqdt
	 *
	 * B = I + qx + q q^T
	 * B^-1 = (1 + q^t q)^(-1) (I - qx)
	 *
	 * dOm = dOm_dq *dq + dOm_d dqdt * d dqdt
	 */

	arma::mat cQ = arma::zeros(3,3);
	cQ(0,1) = -q(2,0);
	cQ(1,0) =  q(2,0);

	cQ(0,2) =  q(1,0);
	cQ(2,0) = -q(1,0);

	cQ(1,2) = -q(0,0);
	cQ(2,1) =  q(0,0);

	arma::mat B  = arma::eye(3,3) + cQ + q*q.t();
	arma::mat Bi = (arma::eye(3,3) + cQ)/(1 + arma::dot(q.col(0),q.col(0)));

	arma::mat dOm_ddqdt = Bi;

	arma::mat dBdqOm = arma::zeros(3,3);

	double q1 = q(0,0);
	double q2 = q(1,0);
	double q3 = q(2,0);

	double Om1 = omega(0,0);
	double Om2 = omega(1,0);
	double Om3 = omega(2,0);

	dBdqOm(0,0) = 2*q1*Om1 + q2*Om2 + q3*Om3;
	dBdqOm(1,0) =   q2*Om1 - Om3;
	dBdqOm(2,0) =   q3*Om1 + Om2;

	dBdqOm(0,1) =   q1*Om2 + Om3;
	dBdqOm(1,1) =   q1*Om1 + 2*q2*Om2 + q3*Om3;
	dBdqOm(2,1) =   -Om1 + q3*Om2;

	dBdqOm(0,2) =   -Om2 + q1*Om3;
	dBdqOm(1,2) =    q1  + q2*Om3;
	dBdqOm(2,2) =   q1*Om1 + q2*Om2 + 2*q3*Om3;

	arma::mat dOmdq = -Bi*dBdqOm;

	cov_omega = arma::zeros(3,3);
	cov_omega = dOmdq*cov_q*dOmdq.t() + dOm_ddqdt*cov_dq*dOm_ddqdt.t();

	dOmdq.print("dOmdq");
	dOm_ddqdt.print("dOmddq");
	cov_q.print("dq");
	cov_dq.print("covdq");


}

dq2omega::~dq2omega() {
	// TODO Auto-generated destructor stub
}

