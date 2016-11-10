/*
 * poseEKF.h
 *
 *@note Extended Kalman Filter for pose estimation
 *
 *  Created on: Nov 8, 2016
 *      Author: xwong
 */

#ifndef POSEEKF_H_
#define POSEEKF_H_

#include <armadillo>
#include "utility.h"
#include "ppTransEst.h"

class poseEKF {
private:

	utility UT;
	ppTransEst ppT;

	arma::mat Xk;
	arma::mat Xkm1;

	arma::mat Sigmakm1;
	arma::mat Sigmak;


public:
	poseEKF();

	bool propagate(arma::mat matA, arma::mat matB, arma::mat SigmaA, arma::mat SigmaB, double dt,arma::mat& propX, arma::mat& propSigmaX);

	void update(arma::mat Xkp, arma::mat SigKp, std::vector<arma::mat> vctPk,std::vector<arma::mat> vctSigPk,
			std::vector<arma::mat> vctPkm1, arma::mat& Xk, arma::mat& SigXk);

	arma::mat jacobianMat(arma::mat q,  std::vector<arma::mat> vctPk);

	virtual ~poseEKF();
};

#endif /* POSEEKF_H_ */
