/*
 * ppTransCov.h
 *
 *  Created on: Oct 14, 2016
 *      Author: xwong
 */

#ifndef PPTRANSCOV_H_
#define PPTRANSCOV_H_

#include "utility.h"

class ppTransCov {
private:
	utility UT;

	/// Eta = P(k+n) - P(k)
	/// Rho = P(k+n) + P(k);

	arma::mat dEtadPTemplate;

	arma::mat dRhodPx;
	arma::mat dRhodPy;
	arma::mat dRhodPz;


public:
	ppTransCov();

	bool update(arma::mat matH,  std::vector< std::vector<cv::Vec3f> > vctVctPt,  std::vector< std::vector<arma::mat> > vctVctCovPt,arma::mat x,double dt, arma::mat& sigmaA, arma::mat& sigmaB );

	virtual ~ppTransCov();

private:

	/*
	 *@note compute the jacobian matrix dXdPk
	 *@input matH ppTrans solution's H matrix
	 *@input vctPt vector of points belonging to Pk
	 *@input x vector of estimate kinematic model coefficient.
	 *@input tIndex time index, 0, 1, 2, 3 - > t = k, t = k - 1, t = k -2, t = k - 3
	 *@input numOfSt number of frame min 2, max 4
	 *@return a 6*(numOfSt-1)x(3n) block matrix of dXdPk, n = vctPt.size();
	 */
	arma::mat compdXdP(arma::mat matH, std::vector<cv::Vec3f> vctPt,arma::mat x,double dt, int tIndex, int numOfSt);



};

#endif /* PPTRANSCOV_H_ */
