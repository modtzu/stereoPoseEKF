/*
 * keypointsManager.cpp
 *
 *  Created on: Oct 14, 2016
 *      Author: xwong
 */

#include "keypointsManager.h"

//keypointsManager::keypointsManager() {
//	// TODO Auto-generated constructor stub
//
//}

void keypointsManager::update(std::vector<cv::KeyPoint>* ptrVctFtL,
								std::vector<cv::KeyPoint>* ptrVctFtR,
								std::vector<arma::mat>* ptrVctCovFtL,
								std::vector<arma::mat>*  ptrVctCovFtR,
								std::vector<int>* ptrRmvID,
								bool resetFlg) {

	if(resetFlg)
	{
//		vctPkm1.clear();
//		vctPkm2.clear();
//		vctPkm3.clear();
//		vctCovPkm1.clear();
//		vctCovPkm2.clear();
//		vctCovPkm3.clear();

		vctPkStr[1].clear();
		vctPkStr[2].clear();
		vctPkStr[3].clear();

		vctCovPkStr[1].clear();
		vctCovPkStr[2].clear();
		vctCovPkStr[3].clear();
	}
	else
	{
		updateStorage(ptrRmvID);

//		vctPkm3 = vctPkm2;
//		vctPkm2 = vctPkm1;
//		vctPkm1 = vctPk;
//
//		vctCovPkm3 = vctCovPkm2;
//		vctCovPkm2 = vctCovPkm1;
//		vctCovPkm1 = vctCovPk;

		vctPkStr[3] = vctPkStr[2];
		vctPkStr[2] = vctPkStr[1];
		vctPkStr[1] = vctPkStr[0];

		vctCovPkStr[3] = vctCovPkStr[2];
		vctCovPkStr[2] = vctCovPkStr[1];
		vctCovPkStr[1] = vctCovPkStr[0];

	}

//	strSolver.computeDepthWtCov(ptrVctFtL,ptrVctFtR,vctPk,ptrVctCovFtL,ptrVctCovFtR,vctCovPk);


	strSolver.computeDepthWtCov(ptrVctFtL,ptrVctFtR,vctPkStr[0],ptrVctCovFtL,ptrVctCovFtR,vctCovPkStr[0]);

}

bool keypointsManager::getVctPk(std::vector<cv::Vec3f>& vctP,
		std::vector<arma::mat>& vctCovP) {

//	if(vctPk.empty())
//		return false;
//
//	vctP = vctPk;
//	vctCovP = vctCovPk;

	if(vctPkStr[0].empty())
		return false;

	vctP = vctPkStr[0];
	vctCovP = vctCovPkStr[0];

	return true;

}

bool keypointsManager::getVctPkm1(std::vector<cv::Vec3f>& vctP,
		std::vector<arma::mat>& vctCovP) {

//	if(vctPkm1.empty())
//		return false;
//
//	vctP = vctPkm1;
//	vctCovP = vctCovPkm1;

	if(vctPkStr[1].empty())
		return false;

	vctP = vctPkStr[1];
	vctCovP = vctCovPkStr[1];

	return true;

}

bool keypointsManager::getVctPkm2(std::vector<cv::Vec3f>& vctP,
		std::vector<arma::mat>& vctCovP) {

//	if(vctPkm2.empty())
//		return false;
//
//	vctP = vctPkm2;
//	vctCovP = vctCovPkm2;

	if(vctPkStr[2].empty())
		return false;

	vctP = vctPkStr[2];
	vctCovP = vctCovPkStr[2];

	return true;

}

bool keypointsManager::getVctPkm3(std::vector<cv::Vec3f>& vctP,
		std::vector<arma::mat>& vctCovP) {

//	if(vctPkm3.empty())
//		return false;
//
//	vctP = vctPkm3;
//	vctCovP = vctCovPkm3;

	if(vctPkStr[3].empty())
		return false;

	vctP = vctPkStr[3];
	vctCovP = vctCovPkStr[3];

	return true;

}


void keypointsManager::updateStorage(std::vector<int>* ptrRmvID) {

	int RC = ptrRmvID->size();

	for(int i=0 ; i < RC; i++)
	{
			int idToRemove = ptrRmvID->at(RC - i - 1);

			if(!vctPkStr[0].empty())
				vctPkStr[0].erase(vctPkStr[0].begin()+idToRemove);

			if(!vctPkStr[1].empty())
				vctPkStr[1].erase(vctPkStr[1].begin()+idToRemove);

			if(!vctPkStr[2].empty())
				vctPkStr[2].erase(vctPkStr[2].begin()+idToRemove);

			if(!vctPkStr[3].empty())
				vctPkStr[3].erase(vctPkStr[3].begin()+idToRemove);
	}

}


keypointsManager::~keypointsManager() {
	// TODO Auto-generated destructor stub
}

void keypointsManager::switch_storage(int ID0, int ID1) {

	/// keyPoints storage
	std::vector<cv::Vec3f> vctPktemp;

	/// keypoints covariance storage
	std::vector<arma::mat> vctCovPktemp;

	vctPktemp = vctPkStr[ID0];
	vctCovPktemp = vctCovPkStr[ID0];

	vctPkStr[ID0] = vctPkStr[ID1];
	vctCovPkStr[ID0] = vctCovPkStr[ID1];

	vctPkStr[ID1] = vctPktemp;
	vctCovPkStr[ID1] = vctCovPktemp;
}
