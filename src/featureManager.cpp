/*
 * featureManager.cpp
 *
 *  Created on: Oct 12, 2016
 *      Author: xwong
 */

#include "featureManager.h"

featureManager::featureManager() {
	// TODO Auto-generated constructor stub
	covTh = 3;
	ptrFeature = std::make_shared<imgFtRelated>(200, 3, 0.0001,10,1.6);
}

featureManager::featureManager(float covTh, int numOfFt) {

	this->covTh = covTh;
	ptrFeature = std::make_shared<imgFtRelated>(numOfFt, 3, 0.0001,10,1.6);
}

int featureManager::initFeaturePair(cv::Mat imgL, cv::Mat imgR) {

	vctFt0L.clear();
	vctFt0R.clear();
	vctCovFt0L.clear();
	vctCovFt0R.clear();

	ptrFeature->getFeature(imgL,vctFt0L,des0,1);

	/// Establish key points correspondence in initial stereo pair with KLT tracker
	ptrFeature->trackFeature(imgL,imgR,vctFt0L,vctFt0R,vctReID,cv::Size(10,10));

	/// get SIFT descriptor for tracked feature
	ptrFeature->getDescriptorForPt(imgL,vctFt0L,des0);
	ptrFeature->getDescriptorForPt(imgR,vctFt0R,des1);

	/// descriptor matching
	cv::Mat dDes = des1 - des0;

	std::vector<cv::KeyPoint> vctFt1L, vctFt1R;

	for (int i = 0; i < dDes.rows; i++)
	{
		float d = cv::norm(dDes.row(i));
		if(d < 200)
		{
			vctFt1L.push_back(vctFt0L[i]);
			vctFt1R.push_back(vctFt0R[i]);
		}

	}

	vctFt0L = vctFt1L;
	vctFt0R = vctFt1R;

	/// descriptor matching for remove false track / outlier
//	ptrFeature->matchFeature(imgL,imgR,vctFt0L,vctFt0R,Pt0,Pt1,des0,des1);

	arma::mat initCov(2,2);
	initCov.eye();

	int CS = vctFt0L.size();

	vctCovFt0L = std::vector<arma::mat>(CS,initCov);
	vctCovFt0R = std::vector<arma::mat>(CS,initCov);

//	for(int i=0; i<CS; i++)
//	{
////		int k = CS - i - 1;
////		arma::mat tCov = KLTU.kltCovLS(imgL0,imgR0,cv::Size(20,20),vctFt0L[k],vctFt0R[k],initCov,
////															cv::Size(5,5),
////															1.6);
//		vctCovFt0L.push_back(initCov);
//		vctCovFt0R.push_back(initCov);
//
////		vctCovFt0R.push_back(tCov);
//	}

	imgL.copyTo(imgL0);
	imgR.copyTo(imgR0);

	return CS;
}

int featureManager::updateFeature(cv::Mat imgLt, cv::Mat imgRt,
		std::vector<int>& ejectID) {

		cv::Size winsize(20,20);

		std::vector<cv::KeyPoint> vctFt1L, vctFt1R;
		std::vector<arma::mat> vctCovFt1L, vctCovFt1R;

		vctCovFt1L = vctCovFt0L;
		vctCovFt1R = vctCovFt0R;

		std::vector<int> rmvID;

		ptrFeature->trackFeature(imgL0,imgLt,vctFt0L,vctFt1L,rmvID,winsize,1);

		int ID2rmv = -1;

		if(!rmvID.empty())
			ID2rmv = 0;

		std::vector<int> rmvID2;

		/// Compute feature covariance and eject if uncertainty is large

		for(int k=0; k<vctFt1L.size(); k++)
		{
			if(ID2rmv >=0 && k == rmvID[ID2rmv])
			{
				rmvID2.push_back(rmvID[ID2rmv]);
				ID2rmv++;
				continue;
			}

			arma::mat initCov = vctCovFt0L[k];

			arma::mat tCov = KLTU.kltCovLS(imgL0,imgLt,winsize,vctFt0L[k],vctFt1L[k],initCov,
																cv::Size(16,16),
																1.6);

			vctCovFt1L[k]=(tCov);

			double approxEig = arma::trace(tCov);

			arma::cx_vec eigVal;
			arma::cx_mat eigVec;

			arma::eig_gen(eigVal,eigVec,tCov);
			arma::mat eig = arma::real(eigVal);

			if(covTh > 0)
			{
				if(approxEig > covTh*covTh || approxEig <= 0)
				{
					rmvID2.push_back(k);
				}
			}

		}


		rmvID.clear();
		ptrFeature->trackFeature(imgR0,imgRt,vctFt0R,vctFt1R,rmvID,winsize,1);

		int ID2rmv2 = 0;
			ID2rmv = 0;

		std::vector<int> rmvID3;

		for(unsigned int k=0; k<vctFt1R.size(); k++)
		{
			if(!rmvID2.empty())
			{
				if(!rmvID.empty())
				{
					if(rmvID2[ID2rmv2]<rmvID[ID2rmv])
					{
						if(k == rmvID2[ID2rmv2])
						{
							rmvID3.push_back(rmvID2[ID2rmv2]);
							ID2rmv2++;
							continue;
						}
					}
					else
					{
						if(k == rmvID[ID2rmv])
						{
							rmvID3.push_back(rmvID[ID2rmv]);
							ID2rmv++;
							continue;
						}
					}
				}
				else
				{
					if(k == rmvID2[ID2rmv2])
					{
						rmvID3.push_back(rmvID2[ID2rmv2]);
						ID2rmv2++;
						continue;
					}
				}
			}
			else
			{
				if(!rmvID.empty())
				{
						if(k == rmvID[ID2rmv])
						{
							rmvID3.push_back(rmvID[ID2rmv]);
							ID2rmv++;
							continue;
						}
				}
			}


			arma::mat initCov = vctCovFt0L[k];

			arma::mat tCov = KLTU.kltCovLS(imgR0,imgRt,winsize,vctFt0R[k],vctFt1R[k],initCov,
																cv::Size(5,5),
																1.6);

			vctCovFt1R[k]=(tCov);

			double approxEig = (arma::trace(tCov));

			if(covTh > 0)
				if(approxEig > covTh*covTh || approxEig <= 0)
				{
					rmvID3.push_back(k);
				}
		}

		int CS  = rmvID3.size();
		for(int i=0; i < CS; i++)
		{
			int k = CS - i - 1;

			int idToRemove = rmvID3[k];

			vctFt1R.erase(vctFt1R.begin()+idToRemove);
			vctFt1L.erase(vctFt1L.begin()+idToRemove);
			vctCovFt1L.erase(vctCovFt1L.begin()+idToRemove);
			vctCovFt1R.erase(vctCovFt1R.begin()+idToRemove);

		}

		imgLt.copyTo(imgL0);
		imgRt.copyTo(imgR0);

		vctFt0R = vctFt1R;
		vctFt0L = vctFt1L;

		vctCovFt0R = vctCovFt1R;
		vctCovFt0L = vctCovFt1L;

		ejectID = rmvID3;

		return vctFt0R.size();
}

void featureManager::getFeaturePair(std::vector<cv::KeyPoint>* ptrFtL,
		std::vector<cv::KeyPoint>* ptrFtR, std::vector<arma::mat>* ptrCovFtL,
		std::vector<arma::mat>* ptrCovFtR) {

		*ptrFtL = vctFt0L;
		*ptrFtR = vctFt0R;
		*ptrCovFtL = vctCovFt0L;
		*ptrCovFtR = vctCovFt0R;
}

featureManager::~featureManager() {
	// TODO Auto-generated destructor stub
}

