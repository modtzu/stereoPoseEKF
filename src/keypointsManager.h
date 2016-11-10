/*
 * keypointsManager.h
 *
 *  Created on: Oct 14, 2016
 *      Author: xwong
 *
 *  estimate and managing keypoints location from feature input
 */

#ifndef KEYPOINTSMANAGER_H_
#define KEYPOINTSMANAGER_H_

#include "featureManager.h"
#include "stereoSolver.h"
#include "utility.h"

class keypointsManager {
private:

	stereoSolver strSolver;

	/// keyPoints storage
	std::vector<cv::Vec3f> vctPk,vctPkm1,vctPkm2, vctPkm3;

	/// keypoints covariance storage
	std::vector<arma::mat> vctCovPk,vctCovPkm1,vctCovPkm2,vctCovPkm3;

	utility UT;
public:
//	keypointsManager();

	/*
	 *note constructor with stereo setting
	 */
	keypointsManager(double u0, double v0, double f, double B)
	{
		strSolver.setStereoParam( u0,  v0,  f,  B);
	}

	/*
	 *@note update manager with new feature points
	 *@input ptrVctFtL pointer to left image feature points
	 *@input ptrVctFtR pointer to right image feature points
	 *@input ptrRmvID pointer to removeID generated from feature manager
	 */
	void update(std::vector<cv::KeyPoint>* ptrVctFtL,std::vector<cv::KeyPoint>* ptrVctFtR,
					std::vector<arma::mat>* ptrVctCovFtL,std::vector<arma::mat>*  ptrVctCovFtR,
						std::vector<int>* ptrRmvID,
						bool resetFlg = false);

	/*
	 *@note extract estimate keypoints at time t
	 *@input
	 *@input
	 *@return false if do not exist
	 */
	bool getVctPk(std::vector<cv::Vec3f>& vctP, std::vector<arma::mat>& vctCovP);

	/*
	 *@note extract estimate keypoints at time t-1
	 *@input
	 *@input
	 *@return false if do not exist
	 */
	bool getVctPkm1(std::vector<cv::Vec3f>& vctP, std::vector<arma::mat>& vctCovP);

	/*
	 *@note extract estimate keypoints at time t-2
	 *@input
	 *@input
	 *@return false if do not exist
	 */
	bool getVctPkm2(std::vector<cv::Vec3f>& vctP, std::vector<arma::mat>& vctCovP);

	/*
	 *@note extract estimate keypoints at time t-3
	 *@input
	 *@input
	 *@return false if do not exist
	 */
	bool getVctPkm3(std::vector<cv::Vec3f>& vctP, std::vector<arma::mat>& vctCovP);

	virtual ~keypointsManager();

private:

	void updateStorage(std::vector<int>* ptrRmvID);

};

#endif /* KEYPOINTSMANAGER_H_ */
