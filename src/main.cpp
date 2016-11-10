/*
 * main.cpp
 *
 *  Created on: Sep 8, 2016
 *      Author: xwong
 *
 * 	estimate relative pose from stereo observation and create a global map
 *
 */

#include "input.h"
#include "imgFtRelated.h"
#include "pcVIsual.h"
#include "stereoSolver.h"
#include "ppTransEst.h"
#include "pclStereo.h"

#include "kltUncertainty.h"
#include "utility.h"

#include "featureManager.h"
#include "keypointsManager.h"

#include "poseEKF.h"

ppTransEst ppTrans;
pcVIsual vi;
kltUncertainty KLTU;
utility UT;
pclStereo pclStr;
poseEKF EKF;

/// argument parser
void argumentParser(int argc, char** argv, std::string& fileExt, std::string& fileName,
		std::string& camCalibfile,std::string& optionFile, int& initCounter)
{

	for(int i =0; i < argc; i++)
	{
		if(strcmp("-e",argv[i])== 0)
		{
			fileExt = std::string(argv[i+1]);
		}
		else
		if(strcmp("-f",argv[i])== 0)
		{
			fileName = std::string(argv[i+1]);
		}
		else
		if(strcmp("-c",argv[i])== 0)
		{
			camCalibfile = std::string(argv[i+1]);
		}
		else
		if(strcmp("-o",argv[i])== 0)
		{
			optionFile = std::string(argv[i+1]);
		}
		else
		if(strcmp("-i",argv[i])== 0)
		{
			initCounter = atoi(argv[i+1]);
		}
	}
}


int main(int argc, char** argv)
{
	std::string fileExt, fileName, camCalibFile, optionFile;
	int initCounter = 0;

	/// parse argument
	argumentParser(argc, argv,fileExt,fileName, camCalibFile,optionFile,initCounter);

	/// setup image input reader
	input imgIn(fileExt,initCounter);

	/// Image at t = k-1
	cv::Mat imgR0,imgL0;

	/// get image at t = 0
	if(!imgIn.update(imgL0,imgR0))
	{
		std::cout<<"Error reading image input\n";
		return false;
	}

	/// feature manager
	featureManager ftMng(10,100);

	/// feature storage
	std::vector<cv::KeyPoint> vctFt0L, vctFt0R;
	std::vector<arma::mat> vctCovFt0L, vctCovFt0R;
	std::vector<int> rmvID;

	int numOfFt = ftMng.initFeaturePair(imgL0,imgR0);

	if(numOfFt>0)
		ftMng.getFeaturePair(&vctFt0L,&vctFt0R,&vctCovFt0L,&vctCovFt0R);
	else
		return false;

	/*
	 * Stereo camera setting
	 */
	double u0 = 305.408;
	double v0 = 244.826;
	double f = 808.083;
	double b = 0.240272;

	/*
	 * Compute 3D map at initial frame
	 */
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudPtr( new pcl::PointCloud<pcl::PointXYZRGB> );

	pclStr.solveStereo(imgL0,imgR0,pointCloudPtr,u0,v0,f,b);

	vi.addPt2Window("fullMap",pointCloudPtr,"0");


	/*
	 * Initialize keypoint manager and estimate keypoints 3D loc. in initial frame
	 */
	keypointsManager keyPtMng(u0,v0,f,b);

	keyPtMng.update(&vctFt0L,&vctFt0R,&vctCovFt0L,&vctCovFt0R,&rmvID);

	/*
	 *  relative pose estimation
	 */

	/// Image at t = k
	cv::Mat imgRt,imgLt;

	arma::mat Rt0, Tt0;
	Rt0.eye(3,3);
	Tt0.zeros(3,1);
	int c = 0;
	int Fr = 0;

	std::vector<arma::mat> vctCamTransHist;
	std::vector<arma::mat> vctCamRotHist;

	double dt = 0.1;

	/// loop through input images sequence
	while(true)
	{
		/// get next image
		if(!imgIn.update(imgLt,imgRt))
		{
			std::cout<<"End of image sequence\n";
			break;
		}

		Fr ++;

		///Track feature from L0 - Lt, R0 - Rt
		std::vector<cv::KeyPoint> vctFt1L, vctFt1R;
		std::vector<arma::mat> vctCovFt1L, vctCovFt1R;
		std::vector<int> rmvID;

		int numOfFt = ftMng.updateFeature(imgLt,imgRt,rmvID);

		if(numOfFt>0)
			ftMng.getFeaturePair(&vctFt1L,&vctFt1R,&vctCovFt1L,&vctCovFt1R);
		else
		{
			std::cout<<"No feature\n";
			 break;
		}

		keyPtMng.update(&vctFt1L,&vctFt1R,&vctCovFt1L,&vctCovFt1R,&rmvID);

		arma::mat R, T;
		arma::mat covQ,covT;

		arma::mat matA,matB;
		arma::mat sigmaA,sigmaB;

		std::vector<int> ejectID;

		std::vector<cv::Vec3f> vctPk,vctPkm1,vctPkm2, vctPkm3;
		std::vector<arma::mat> vctCovPk,vctCovPkm1,vctCovPkm2,vctCovPkm3;

		/*
		 * (UPDATE)
		 *  compute 3D point from current stereo images + estimate relative pose to previous frame
		 * 		depending on previously available information, using model in different order.
		 */
		if(keyPtMng.getVctPkm3(vctPkm3,vctCovPkm3))
		{
			keyPtMng.getVctPkm2(vctPkm2,vctCovPkm2);
			keyPtMng.getVctPkm1(vctPkm1,vctCovPkm1);
			keyPtMng.getVctPk(vctPk,vctCovPk);

//			ppTrans.solveLinear3rdOrder(vctPkm3,vctPkm2,vctPkm1,vctPk,R,T,ejectID); /// solve with linear least square 3rd order model

			ppTrans.solveLinear3rdOrderWtCov(vctPkm3,vctPkm2,vctPkm1,vctPk,vctCovPkm3,vctCovPkm2,vctCovPkm1,vctCovPk,matA,matB,sigmaA,sigmaB,ejectID);

			T = matA.col(0) + matA.col(1)*dt + matA.col(2)*dt*dt/2;
			arma::mat q = matB.col(0) + matB.col(1)*dt + matB.col(2)*dt*dt/2;
			R = ppTrans.CRP2ROT(q);

		}
		else if(keyPtMng.getVctPkm2(vctPkm2,vctCovPkm2))
		{
			keyPtMng.getVctPkm1(vctPkm1,vctCovPkm1);
			keyPtMng.getVctPk(vctPk,vctCovPk);

//			ppTrans.solveLinear2ndOrder(vctPkm2,vctPkm1,vctPk,R,T,ejectID); /// solve with linear least square 2nd order model

			ppTrans.solveLinear2ndOrderWtCov(vctPkm2,vctPkm1,vctPk,vctCovPkm2,vctCovPkm1,vctCovPk,matA,matB,sigmaA,sigmaB,ejectID);

			T = matA.col(0) + matA.col(1)*dt;
			arma::mat q = matB.col(0) + matB.col(1)*dt;

			R = ppTrans.CRP2ROT(q);
		}
		else
			if(keyPtMng.getVctPkm1(vctPkm1,vctCovPkm1))
		{
			keyPtMng.getVctPk(vctPk,vctCovPk);
//			ppTrans.solveLinear(vctPkm1,vctPk,R,T,ejectID); /// solve with linear least square 1st order model

			ppTrans.solveLinearWtCov(vctPkm1,vctPk,vctCovPkm1,vctCovPk,matA,matB,sigmaA,sigmaB,ejectID);

			T = matA.col(0);
			arma::mat q = matB.col(0);

			R = ppTrans.CRP2ROT(q);
		}
		else
		{
			std::cout<<"Error : no previous information available\n";
			break;
		}

		double dt = 0.1;

		covQ = ppTrans.sigmaB2SigmaQ(sigmaB,dt);
		covT = ppTrans.sigmaA2SigmaT(sigmaA,dt);

//		covQ.print("covQ");
//		covT.print("covT");

	 /// transform estimated relative pose back into initial frame
		T = -R.t()*T;
		R = R.t();

		Tt0 = Tt0 + Rt0*T;
		Rt0 = R*Rt0;

		vctCamTransHist.push_back(Tt0);
		vctCamRotHist.push_back(Rt0);

	/// print camera relative position to initial frame
		std::cout<<vctPk.size()<<","<<Tt0.t();

		if(c == 3 || c > 20)
		{
			/// visualize key points in initial frame.
			Eigen::Matrix4f transformMat = Eigen::Matrix4f::Identity();

			for(int i=0 ; i < 3; i ++)
			{
				for(int j=0 ; j < 3; j ++)
				{
					transformMat(i,j) = Rt0(i,j);
				}
				transformMat(i,3) = Tt0(i,0);
			}

			char ptName[256];
			sprintf(ptName,"%d",Fr);

			pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudPtr( new pcl::PointCloud<pcl::PointXYZRGB> );

			pclStr.solveStereo(imgL0,imgR0,pointCloudPtr,u0,v0,f,b);

			vi.addPt2Window("fullMap",pointCloudPtr,std::string(ptName),transformMat);

			if( c > 10)
				c = 4;
		}


		/// visualize tracked feature points in each image
		cv::Mat kf, kfR;
		cv::drawKeypoints(imgLt,vctFt1L,kf,cv::Scalar(0,255,255));
		cv::drawKeypoints(imgRt,vctFt1R,kfR,cv::Scalar(0,255,255));
		cv::imshow("kfR",kfR);
		cv::imshow("kf",kf);

//		cv::imwrite("Rft.png",kfR);
//		cv::imwrite("Lft.png",kf);

		char k = cv::waitKey(1);

		if(k == 27)
			break;

		/// update image
		imgRt.copyTo(imgR0);
		imgLt.copyTo(imgL0);

		/// if number of feature is least than 20, reset feature
		if(vctPk.size()< 20)
		{
			int numOfFt = ftMng.initFeaturePair(imgL0,imgR0);

			if(numOfFt>0)
				ftMng.getFeaturePair(&vctFt0L,&vctFt0R,&vctCovFt0L,&vctCovFt0R);
			else
			{
				std::cout<<"Error: no feature detected\n";
				break;
			}

			keyPtMng.update(&vctFt0L,&vctFt0R,&vctCovFt0L,&vctCovFt0R,&rmvID,true);

			c = 0;

		}
		else
		{
			c++;
		}


	}

	vi.showWindow("fullMap");

	return 0;

	/// relative pose uncertainty

	/// implement Kalman filter for subsequence state & covariance estimation
		/// Update / Propagate


	/// Map / pose uncertainty

}
