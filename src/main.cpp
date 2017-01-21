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

#include "utility.h"
#include "plotData.h"

#include "kltUncertainty.h"
#include "featureManager.h"
#include "keypointsManager.h"

#include "poseEKF.h"

#include <fstream>
#include <iostream>

#include "dq2omega.h"


ppTransEst ppTrans;
pcVIsual vi;
kltUncertainty KLTU;
utility UT;
pclStereo pclStr;
poseEKF EKF;
plotData pl;

dq2omega dqdOm;


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
	featureManager ftMng(3,300);

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
	vi.addCamera2Window("fullMap","cam0",Eigen::Matrix4f::Identity());

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

	std::vector<arma::mat> vctTransCov;
	std::vector<arma::mat> vctCRPCov;

	std::vector<arma::mat> vctRefTrans;
	std::vector<arma::mat> vctRefCRP;

	std::vector<float> sigTx;
	std::vector<float> sigTy;
	std::vector<float> sigTz;

	std::vector<float> sigQx;
	std::vector<float> sigQy;
	std::vector<float> sigQz;


	double dt = 0.1;

	arma::mat propX = arma::zeros(1,6), propSigX = arma::zeros(6,6);

	arma::mat propXp = propX, propSigXp = propSigX;

	arma::mat preA, preB, preSigA, preSigB;

	int tri =0;

	int initF = numOfFt;

	std::ofstream writerQvel, writerTvel;
	std::ofstream writerQvelCov, writerTvelCov;

	std::ofstream writerOmCov, writerOm;

	writerQvel.open("refQvel.txt");
	writerTvel.open("refTvel.txt");

	writerQvelCov.open("refQvelCov.txt");
	writerTvelCov.open("refTvelCov.txt");

	writerOmCov.open("Om.txt");
	writerOm.open("Om_Cov.txt");

	/// loop through input images sequence
	while(true)
	{
		tri ++;

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

		arma::mat R, T, q;
		arma::mat covQ,covT;

		arma::mat v_T, v_q;
		arma::mat cov_vq,cov_vT;


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

//			ppTrans.solveLinear3rdOrderWtCov(vctPkm3,vctPkm2,vctPkm1,vctPk,vctCovPkm3,vctCovPkm2,vctCovPkm1,vctCovPk,matA,matB,sigmaA,sigmaB,ejectID);

			ppTrans.solveLinear3rdOrderWtCov(vctPk,vctPkm1,vctPkm2,vctPkm3,vctCovPk,vctCovPkm1,vctCovPkm2,vctCovPkm3,matA,matB,sigmaA,sigmaB,ejectID);

			T = matA.col(0) + matA.col(1)*dt + matA.col(2)*dt*dt/2;
			q = matB.col(0) + matB.col(1)*dt + matB.col(2)*dt*dt/2;
			R = ppTrans.CRP2ROT(q);

			v_T = matA.col(1);
			v_q = matB.col(1);

			cov_vT = sigmaA.submat(3,3,5,5);
			cov_vq = sigmaB.submat(3,3,5,5);;
		}
		else
			if(keyPtMng.getVctPkm2(vctPkm2,vctCovPkm2))
		{
			keyPtMng.getVctPkm1(vctPkm1,vctCovPkm1);
			keyPtMng.getVctPk(vctPk,vctCovPk);

//			ppTrans.solveLinear2ndOrder(vctPkm2,vctPkm1,vctPk,R,T,ejectID); /// solve with linear least square 2nd order model

//			ppTrans.solveLinear2ndOrderWtCov(vctPkm2,vctPkm1,vctPk,vctCovPkm2,vctCovPkm1,vctCovPk,matA,matB,sigmaA,sigmaB,ejectID);

			ppTrans.solveLinear2ndOrderWtCov(vctPk,vctPkm1,vctPkm2,vctCovPk,vctCovPkm1,vctCovPkm2,matA,matB,sigmaA,sigmaB,ejectID);

			T = matA.col(0) + matA.col(1)*dt;
			q = matB.col(0) + matB.col(1)*dt;

			R = ppTrans.CRP2ROT(q);

			v_T = matA.col(1);
			v_q = matB.col(1);

			cov_vT = sigmaA.submat(3,3,5,5);
			cov_vq = sigmaB.submat(3,3,5,5);;
		}
		else
			if(keyPtMng.getVctPkm1(vctPkm1,vctCovPkm1))
		{
			keyPtMng.getVctPk(vctPk,vctCovPk);
//			ppTrans.solveLinear(vctPkm1,vctPk,R,T,ejectID); /// solve with linear least square 1st order model

			ppTrans.solveLinearWtCov(vctPk,vctPkm1,vctCovPk,vctCovPkm1,matA,matB,sigmaA,sigmaB,ejectID);

			T = matA.col(0);
			q = matB.col(0);

			R = ppTrans.CRP2ROT(q);

			v_T = arma::zeros(3,1);
			v_q = arma::zeros(3,1);

			cov_vT = arma::zeros(3,3);
			cov_vq = arma::zeros(3,3);

		}
		else
		{
			std::cout<<"Error : no previous information available\n";
			break;
		}

		covQ = ppTrans.sigmaB2SigmaQ(sigmaB,dt);
		covT = ppTrans.sigmaA2SigmaT(sigmaA,dt);

		propXp = propX;
		propSigXp = propSigX;

		EKF.propagate(matA,matB,sigmaA,sigmaB,dt,propX,propSigX);

//		covQ.print("covQ");
//		covT.print("covT");

		arma::mat pQ,pT;
		pQ = covQ;
		pT = covT;

		if(arma::norm(propXp)!=0)
		{
			arma::mat Xk, sigXk;

			arma::mat Xm(6,1);
			Xm.submat(0,0,2,0) = T;
			Xm.submat(3,0,5,0) = q;

			arma::mat sigXm = arma::zeros(6,6);
			sigXm.submat(0,0,2,2) = covT;
			sigXm.submat(3,3,5,5) = covQ;

//			EKF.updateMkII(propXp,propSigXp,Xm,sigXm,Xk,sigXk);

			EKF.update(propXp, propSigXp,
						vctPk, vctCovPk,
						vctPkm1, Xk,sigXk);

			T = Xk.submat(0,0,2,0);
			q = Xk.submat(3,0,5,0);

			covT = sigXk.submat(0,0,2,2);
			covQ = sigXk.submat(3,3,5,5);

			R = ppTrans.CRP2ROT(q);
		}

		vctRefTrans.push_back(T);
		vctRefCRP.push_back(q);

		writerQvel<<v_q(0,0)<<","<<v_q(1,0)<<","<<v_q(2,0)<<"\n";
		writerTvel<<v_T(0,0)<<","<<v_T(1,0)<<","<<v_T(2,0)<<"\n";

		writerQvelCov<<sqrt(cov_vq(0,0))<<","<<sqrt(cov_vq(1,1))<<","<<sqrt(cov_vq(2,2))<<"\n";
		writerTvelCov<<sqrt(cov_vT(0,0))<<","<<sqrt(cov_vT(1,1))<<","<<sqrt(cov_vT(2,2))<<"\n";

		arma::mat om, cov_om;

		dqdOm.convert(q,v_q,om);

		dqdOm.compute_cov_omega(q,v_q,om,covQ,cov_vq,cov_om);

		writerOmCov<<om(0,0)<<","<<om(1,0)<<","<<om(2,0)<<"\n";
		writerOm<<sqrt(cov_om(0,0))<<","<<sqrt(cov_om(1,1))<<","<<sqrt(cov_om(2,2))<<"\n";

//		if(sigTx.size()>1)
//		std::cout<<sqrt(covT(0,0))<<" "<<sqrt(covT(0,0))-sigTx[sigTx.size()-1]<<"\n";


//		pQ.print("pQ");
//		covQ.print("cQ");
//		pT.print("pT");
//		covT.print("cT");

		sigTx.push_back(sqrt(covT(0,0)));
		sigTy.push_back(sqrt(covT(1,1)));
		sigTz.push_back(sqrt(covT(2,2)));

		sigQx.push_back(sqrt(covQ(0,0)));
		sigQy.push_back(sqrt(covQ(1,1)));
		sigQz.push_back(sqrt(covQ(2,2)));

		if(vctTransCov.empty())
		{
			vctTransCov.push_back(covT);
			vctCRPCov.push_back(covQ);
		}
		else
		{
			vctTransCov.push_back(covT + vctTransCov[vctTransCov.size()-1]);
			vctCRPCov.push_back(covQ + vctCRPCov[vctCRPCov.size()-1]);
		}

//		sigTx.push_back(sqrt(vctTransCov[vctTransCov.size()-1](0,0)));
//		sigTy.push_back(sqrt(vctTransCov[vctTransCov.size()-1](1,1)));
//		sigTz.push_back(sqrt(vctTransCov[vctTransCov.size()-1](2,2)));
//
//		sigQx.push_back(sqrt(vctCRPCov[vctCRPCov.size()-1](0,0)));
//		sigQy.push_back(sqrt(vctCRPCov[vctCRPCov.size()-1](1,1)));
//		sigQz.push_back(sqrt(vctCRPCov[vctCRPCov.size()-1](2,2)));



//		propSigXp.submat(0,0,2,2).print("sigT");
//		propSigXp.submat(3,3,5,5).print("sigQ");

//		T.t().print("estT");
//		q.t().print("estQ");

	 /// transform estimated relative pose back into initial frame
		T = -R.t()*T;
		R = R.t();

		Tt0 = Tt0 + Rt0*T;
		Rt0 = R*Rt0;

		vctCamTransHist.push_back(Tt0);
		vctCamRotHist.push_back(Rt0);

	/// print camera relative position to initial frame
		std::cout<<Fr<<","<<vctPk.size()<<","<<Tt0.t();

		if(tri > 20)
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

			char camName[256];
			sprintf(camName,"cam%d",Fr);

			pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudPtr( new pcl::PointCloud<pcl::PointXYZRGB> );

			pclStr.solveStereo(imgLt,imgRt,pointCloudPtr,u0,v0,f,b);

			vi.addPt2Window("fullMap",pointCloudPtr,std::string(ptName),transformMat);

			vi.addCamera2Window("fullMap",std::string(camName),transformMat);

			tri = 0;
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

		/// if number of feature is least than 20, reset feature

		float sc = (float)vctPk.size()/(float)initF;

		if(sc < 0.1 || vctPk.size() < 20)
		{
			/// restart feature at current frame, but set it as second frame
			int numOfFt = ftMng.initFeaturePair(imgLt,imgRt);

			if(numOfFt>0)
				ftMng.getFeaturePair(&vctFt1L,&vctFt1R,&vctCovFt1L,&vctCovFt1R);
//				ftMng.getFeaturePair(&vctFt0L,&vctFt0R,&vctCovFt0L,&vctCovFt0R);
			else
			{
				std::cout<<"Error: no feature detected\n";
				break;
			}

			/// track feature backward 1 frame, and set it as 1st frame
			ftMng.updateFeature(imgL0,imgR0,rmvID);
			ftMng.getFeaturePair(&vctFt0L,&vctFt0R,&vctCovFt0L,&vctCovFt0R);


			keyPtMng.update(&vctFt1L,&vctFt1R,&vctCovFt1L,&vctCovFt1R,&rmvID,true);
			keyPtMng.update(&vctFt0L,&vctFt0R,&vctCovFt0L,&vctCovFt0R,&rmvID);

			keyPtMng.switch_storage(0,1);

			initF = numOfFt;

			c = 0;
		}
		else
		{
			c++;
		}

		/// update image
		imgRt.copyTo(imgR0);
		imgLt.copyTo(imgL0);

	}

	writerQvel.close();
	writerTvel.close();

	writerQvelCov.close();
	writerTvelCov.close();


	pl.plot(sigQx,0,"1-sigma",PL_GRID_ON);
	pl.save("Q1.png");

	pl.plot(sigQy,0,"1-sigma",PL_GRID_ON);
	pl.save("Q2.png");

	pl.plot(sigQz,0,"1-sigma",PL_GRID_ON);
	pl.save("Q3.png");

	pl.plot(sigTx,0,"1-sigma",PL_GRID_ON);
	pl.save("Tx.png");

	pl.plot(sigTy,0,"1-sigma",PL_GRID_ON);
	pl.save("Ty.png");

	pl.plot(sigTz,0,"1-sigma",PL_GRID_ON);
	pl.save("Tz.png");

	vi.showWindow("fullMap");

	std::ofstream writer;

	char sfileName[256];
	sprintf(sfileName,"cov.txt");

	writer.open(sfileName);

	for(int i= 0;i <sigQx.size(); i++)
	{
		writer<<sigQx[i]<<","<<sigQy[i]<<","<<sigQz[i]<<","<<sigTx[i]<<","<<sigTy[i]<<","<<sigTz[i]<<"\n";
	}

	writer.close();

	std::ofstream writerTcov;
	writerTcov.open("covT.txt");

	for(int i= 0;i <vctTransCov.size(); i++)
	{
		writerTcov<<vctTransCov[i](0,0)<<","<<vctTransCov[i](0,1)<<","<<vctTransCov[i](0,2)<<","
				<<vctTransCov[i](1,1)<<","<<vctTransCov[i](1,2)<<","
				<<vctTransCov[i](2,2)<<"\n";
	}

	writerTcov.close();

	std::ofstream writerQcov;
	writerQcov.open("covQ.txt");

	for(int i= 0;i <vctCRPCov.size(); i++)
	{
		writerQcov<<vctCRPCov[i](0,0)<<","<<vctCRPCov[i](0,1)<<","<<vctCRPCov[i](0,2)<<","
				<<vctCRPCov[i](1,1)<<","<<vctCRPCov[i](1,2)<<","
				<<vctCRPCov[i](2,2)<<"\n";
	}

	writerQcov.close();

	std::ofstream writerT;
	writerT.open("refT.txt");

	for(int i= 0;i <vctRefTrans.size(); i++)
	{
		writerT<<vctRefTrans[i](0,0)<<","<<vctRefTrans[i](1,0)<<","<<vctRefTrans[i](2,0)<<"\n";
	}

	writerT.close();

	std::ofstream writerQ;
	writerQ.open("refQ.txt");

	for(int i= 0;i <vctRefCRP.size(); i++)
	{
		writerQ<<vctRefCRP[i](0,0)<<","<<vctRefCRP[i](1,0)<<","<<vctRefCRP[i](2,0)<<"\n";
	}

	writerQ.close();


	return 0;

	/// relative pose uncertainty

	/// implement Kalman filter for subsequence state & covariance estimation
		/// Update / Propagate


	/// Map / pose uncertainty

}
