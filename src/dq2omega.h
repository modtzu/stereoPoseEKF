/*
 * dq2omega.h
 *
 *  Created on: Jan 17, 2017
 *      Author: xwong
 */

#ifndef DQ2OMEGA_H_
#define DQ2OMEGA_H_

#include "armadillo"

class dq2omega {
public:
	dq2omega();

	void convert(arma::mat q, arma::mat dqdt, arma::mat &omega);

	void compute_cov_omega(arma::mat q, arma::mat dqdt, arma::mat omega,
			arma::mat cov_q, arma::mat cov_dq,  arma::mat & cov_omega);

	virtual ~dq2omega();
};

#endif /* DQ2OMEGA_H_ */
