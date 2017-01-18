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

	void update(arma::mat Xkp, arma::mat SigKp, std::vector<cv::Vec3f> vctPk,std::vector<arma::mat> vctSigPk,
			std::vector<cv::Vec3f> vctPkm1, arma::mat& Xk, arma::mat& SigXk);


	void updateMkII(arma::mat Xkp, arma::mat SigXp, arma::mat Xm, arma::mat SigXm, arma::mat& Xk, arma::mat& SigXk);

	arma::mat jacobianMat(arma::mat q,  std::vector<cv::Vec3f> vctPk);

	virtual ~poseEKF();
};

#endif /* POSEEKF_H_ */
