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

dq2omega::~dq2omega() {
	// TODO Auto-generated destructor stub
}

