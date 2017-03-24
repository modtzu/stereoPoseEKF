/*
 * ppTransEst.cpp
 *
 *  Created on: Sep 13, 2016
 *      Author: xwong
 */

#include "ppTransEst.h"

ppTransEst::ppTransEst() {
	// TODO Auto-generated constructor stub

}

arma::mat ppTransEst::system(arma::mat X) {

	arma::mat q;
	q<<X(0,0)<<X(1,0)<<X(2,0)<<arma::endr;
	q = q.t();

	arma::mat qX = crossProductMatrix(q);

	arma::mat t;
	t<<X(3,0)<<X(4,0)<<X(5,0)<<arma::endr;
	t = t.t();

	arma::mat Y(3*nP,1);

	for(int i = 0 ;i < nP; i++)
	{
		int c = 3*i;

		arma::mat pm;
		pm<<Pk(c,0)<<Pk(c+1,0)<<Pk(c+2,0)<<arma::endr;

		arma::mat ePm = qX*pm.t() - t - qX*t;

		Y(c,0) =   ePm(0,0);
		Y(c+1,0) = ePm(1,0);
		Y(c+2,0) = ePm(2,0);
	}

	return Y;
}

void ppTransEst::solve(std::vector<cv::Vec3f> Pt0, std::vector<cv::Vec3f> Pt1,
		arma::mat& rotMat, arma::mat& transT) {

	Pk.zeros(3*Pt0.size(),1);
	Pm.zeros(3*Pt0.size(),1);
	nP = Pt0.size();

	for(int i =0; i < nP; i++)
	{
		int c = 3*i;
		Pk(c,0) = Pt1[i][0] + Pt0[i][0];
		Pk(c+1,0) = Pt1[i][1] + Pt0[i][1];
		Pk(c+2,0) = Pt1[i][2] + Pt0[i][2];

		Pm(c,0) = Pt0[i][0] - Pt1[i][0];
		Pm(c+1,0) = Pt0[i][1] - Pt1[i][1];
		Pm(c+2,0) = Pt0[i][2] - Pt1[i][2];
	}

	arma::mat X;
	X.zeros(6,1);

	int count = 0;

	double ErrP = 1e10;

	while(count < 20)
	{
		arma::mat Y = system(X);
		arma::mat dY = Pm - Y;

//		std::cout<<arma::dot(dY.col(0),dY.col(0))<<"\n";

		double Err= arma::dot(dY.col(0),dY.col(0));

		if(Err < 1e-6)
			break;

		if(fabs(Err-ErrP) < 1e-10)
			break;

		ErrP = Err;

		arma::mat H = jacobianMatix(X);

		arma::mat dX = (H.t()*H).i()*H.t()*dY;

		X = X + dX;

		count++;
	}

	rotMat = CRP2ROT(X.submat(0,0,2,0));
	transT = X.submat(3,0,5,0);
}


bool ppTransEst::solve(std::vector<cv::Vec3f> Pt0, std::vector<cv::Vec3f> Pt1,
		arma::mat& rotMat, arma::mat& transT, std::vector<int>& ransacEjectID) {

	Pk.zeros(3*Pt0.size(),1);
	Pm.zeros(3*Pt0.size(),1);
	nP = Pt0.size();

	for(int i =0; i < nP; i++)
	{
		int c = 3*i;
		Pk(c,0) = Pt1[i][0] + Pt0[i][0];
		Pk(c+1,0) = Pt1[i][1] + Pt0[i][1];
		Pk(c+2,0) = Pt1[i][2] + Pt0[i][2];

		Pm(c,0) = Pt0[i][0] - Pt1[i][0];
		Pm(c+1,0) = Pt0[i][1] - Pt1[i][1];
		Pm(c+2,0) = Pt0[i][2] - Pt1[i][2];
	}

	arma::mat X;
	X.zeros(6,1);

	int count = 0;

	double ErrP = 1e10;

	arma::mat resErr;

	while(count < 20)
	{
		arma::mat Y = system(X);
		arma::mat dY = Pm - Y;

//		std::cout<<arma::dot(dY.col(0),dY.col(0))<<"\n";

		double Err= arma::dot(dY.col(0),dY.col(0));
		resErr = dY;

		if(Err < 1e-6)
			break;

		if(fabs(Err-ErrP) < 1e-10)
			break;

		ErrP = Err;

		arma::mat H = jacobianMatix(X);

		arma::mat dX = (H.t()*H).i()*H.t()*dY;

		X = X + dX;

		count++;
	}

	arma::mat var = arma::var(resErr);

	bool flg = true;

	double sig = sqrt(var(0,0));

	for(int i=0; i < resErr.n_rows; i++)
	{
		if(fabs(resErr(i,0)) > 6*sig)
		{
			ransacEjectID.push_back(i);
			flg = false;
		}
	}

	rotMat = CRP2ROT(X.submat(0,0,2,0));
	transT = X.submat(3,0,5,0);

	return flg;
}

arma::mat ppTransEst::CRP2ROT(arma::mat q) {

	arma::mat qX = crossProductMatrix(q);

	arma::mat I(3,3);
	I.eye();

	arma::mat R = (I +qX).i()*(I-qX);

	return R;
}

arma::mat ppTransEst::jacobianMatix(arma::mat X) {

	arma::mat H(3*nP,6);
	H.zeros();

	arma::mat q;
	q<<X(0,0)<<X(1,0)<<X(2,0)<<arma::endr;
	q = q.t();

	arma::mat t;
	t<<X(3,0)<<X(4,0)<<X(5,0)<<arma::endr;
	t = t.t();

	arma::mat qX = crossProductMatrix(q);
	arma::mat tX = crossProductMatrix(t);

	arma::mat I(3,3);
	I.eye();

	for(int i = 0 ;i < nP; i++)
	{
		int c = 3*i;

		arma::mat Ht(3,6);

		arma::vec pk;
		pk<<Pk(c,0)<<Pk(c+1,0)<<Pk(c+2,0)<<arma::endr;

		arma::mat pkX = crossProductMatrix(pk);

		Ht.submat(0,0,2,2) = -pkX + tX;
		Ht.submat(0,3,2,5) = -I-qX;

		H.submat(c,0,c+2,5) = Ht;
	}

	return H;

}

arma::mat ppTransEst::crossProductMatrix(arma::vec q) {

	arma::mat qX(3,3);

	qX.zeros();
	qX(0,1) = -q(2);
	qX(1,0) = q(2);

	qX(0,2) = q(1);
	qX(2,0) = -q(1);

	qX(1,2) = -q(0);
	qX(2,1) = q(0);

	return qX;
}

arma::mat ppTransEst::sigmaA2SigmaT(arma::mat sigmaA, double dt) {

	arma::mat SigmaT;

	if(sigmaA.n_cols > 8)
	{

		SigmaT = sigmaA.submat(0,0,2,2) + sigmaA.submat(3,3,5,5)*dt*dt + sigmaA.submat(6,6,8,8)*pow(dt,4)/4
					+ 2*sigmaA.submat(0,3,2,5)*dt + sigmaA.submat(0,6,2,8)*dt*dt + sigmaA.submat(3,6,5,8)*pow(dt,3)/2;

	}
	else if(sigmaA.n_cols > 5)
	{
		SigmaT = sigmaA.submat(0,0,2,2) + sigmaA.submat(3,3,5,5)*dt*dt
					+ 2*sigmaA.submat(0,3,2,5)*dt;
	}
	else
	{
		SigmaT = sigmaA;
	}

	return SigmaT;
}

arma::mat ppTransEst::sigmaB2SigmaQ(arma::mat sigmaB, double dt) {

	arma::mat SigmaQ;

	if(sigmaB.n_cols > 8)
	{

		SigmaQ = sigmaB.submat(0,0,2,2) + sigmaB.submat(3,3,5,5)*dt*dt + sigmaB.submat(6,6,8,8)*pow(dt,4)/4
					+ 2*sigmaB.submat(0,3,2,5)*dt + sigmaB.submat(0,6,2,8)*dt*dt + sigmaB.submat(3,6,5,8)*pow(dt,3);


	}
	else if(sigmaB.n_cols > 5)
	{
		SigmaQ = sigmaB.submat(0,0,2,2) + sigmaB.submat(3,3,5,5)*dt*dt
					+ 2*sigmaB.submat(0,3,2,5)*dt;

	}
	else
	{
		SigmaQ = sigmaB;
	}

	return SigmaQ;

}

ppTransEst::~ppTransEst() {
	// TODO Auto-generated destructor stub
}

bool ppTransEst::solveLinear(std::vector<cv::Vec3f> Pt0,
		std::vector<cv::Vec3f> Pt1, arma::mat& rotMat, arma::mat& transT,
		std::vector<int>& ransacEjectID) {

	Pk.zeros(3*Pt0.size(),1);
	Pm.zeros(3*Pt0.size(),1);
	nP = Pt0.size();

	arma::mat H(Pt0.size()*3,6);

	for(int i =0; i < nP; i++)
	{
		int c = 3*i;

		Pm(c,0) = Pt0[i][0] - Pt1[i][0];
		Pm(c+1,0) = Pt0[i][1] - Pt1[i][1];
		Pm(c+2,0) = Pt0[i][2] - Pt1[i][2];

		arma::mat pk(3,1);

		pk(0,0) = Pt1[i][0] + Pt0[i][0];
		pk(1,0) = Pt1[i][1] + Pt0[i][1];
		pk(2,0) = Pt1[i][2] + Pt0[i][2];

		arma::mat pkX = crossProductMatrix(pk);

		H.submat(c,0,c+2,2) = pkX;
		H.submat(c,3,c+2,5).eye();

	}

	arma::mat X = -(H.t()*H).i()*H.t()*Pm;

	arma::mat q = X.submat(0,0,2,0);

	arma::mat Q = crossProductMatrix(q);

	arma::mat I;
	I.eye(3,3);

	arma::mat t = (I + Q).i()*X.submat(3,0,5,0);

	arma::mat Y = H*X;
	arma::mat dY = Pm - Y;

	arma::mat var = arma::var(dY);

	bool flg = true;

	double sig = sqrt(var(0,0));

	for(int i=0; i < dY.n_rows; i++)
	{
		if(fabs(dY(i,0)) > 6*sig)
		{
			ransacEjectID.push_back(i);
			flg = false;
		}
	}

	rotMat = CRP2ROT(q);
	transT = t;

	return flg;
}

bool ppTransEst::solveLinear2ndOrder(std::vector<cv::Vec3f> Pt0,
		std::vector<cv::Vec3f> Pt1, std::vector<cv::Vec3f> Pt2, arma::mat& rotMat, arma::mat& transT,
		std::vector<int>& ransacEjectID) {
//
//	double dt = 0.1;
//	nP = Pt2.size();
//
//	Pm.zeros(nP*3*2,1);
//	arma::mat H(nP*3*2,12);
//	H.zeros();
//
//	for(int i =0; i < nP; i++)
//	{
//		int c = 3*i;
//
//		Pm(c,0) = Pt1[i][0] - Pt2[i][0];
//		Pm(c+1,0) = Pt1[i][1] - Pt2[i][1];
//		Pm(c+2,0) = Pt1[i][2] - Pt2[i][2];
//
//		arma::mat pk(3,1);
//
//		pk(0,0) = Pt2[i][0] + Pt1[i][0];
//		pk(1,0) = Pt2[i][1] + Pt1[i][1];
//		pk(2,0) = Pt2[i][2] + Pt1[i][2];
//
//		arma::mat pkX = crossProductMatrix(pk);
//
//		H.submat(c,0,c+2,2) = pkX;
//		H.submat(c,3,c+2,5) = pkX*dt;
//		H.submat(c,6,c+2,8).eye();
//		H.submat(c,9,c+2,11).eye();
//
//		H.submat(c,9,c+2,11)*=dt;
//	}
//
//	for(int i =0; i < nP; i++)
//	{
//		int c = 3*i + 3*nP;
//
//		Pm(c,0) = Pt0[i][0] - Pt2[i][0];
//		Pm(c+1,0) = Pt0[i][1] - Pt2[i][1];
//		Pm(c+2,0) = Pt0[i][2] - Pt2[i][2];
//
//		arma::mat pk(3,1);
//
//		pk(0,0) = Pt2[i][0] + Pt0[i][0];
//		pk(1,0) = Pt2[i][1] + Pt0[i][1];
//		pk(2,0) = Pt2[i][2] + Pt0[i][2];
//
//		arma::mat pkX = crossProductMatrix(pk);
//
//		H.submat(c,0,c+2,2) = pkX;
//		H.submat(c,3,c+2,5) = 2*pkX*dt;
//
//		H.submat(c,6,c+2,8).eye();
//		H.submat(c,9,c+2,11).eye();
//
//		H.submat(c,9,c+2,11)*=2*dt;
//	}
//
//	arma::mat X = -(H.t()*H).i()*H.t()*Pm;
//
//	arma::mat q = X.submat(0,0,2,0) + X.submat(3,0,5,0)*dt;
//
//	arma::mat Q = crossProductMatrix(q);
//
//	arma::mat C1 = crossProductMatrix(X.submat(0,0,2,0));
//	arma::mat C2 = crossProductMatrix(X.submat(3,0,5,0))*dt;
//
//	arma::mat I;
//	I.eye(3,3);
//
//	arma::mat C = (I + C1 + C2).i();
//
//	arma::mat a1 = C*X.submat(6,0,8,0);
//	arma::mat a2 = C*X.submat(9,0,11,0);
//
//	rotMat = CRP2ROT(q);
//	transT = a1 + a2*dt;
//
//	return true;

	/// EX

	double dt = 0.1;
	nP = Pt2.size();

	Pm.zeros(nP*3*3,1);
	arma::mat H(nP*3*3,12);
	H.zeros();

	std::vector<cv::Vec3f> Str[3];
	Str[0] = Pt1;
	Str[1] = Pt2;

	for(int k = 0; k < 2; k++)
	{
			for(int i =0; i < nP; i++)
			{
				int c = 3*i + k*3*nP;

				int ord = k+1;

				Pm(c,0) = Pt0[i][0] - Str[k][i][0];
				Pm(c+1,0) = Pt0[i][1] - Str[k][i][1];
				Pm(c+2,0) = Pt0[i][2] - Str[k][i][2];

				arma::mat pk(3,1);

				pk(0,0) = Str[k][i][0] + Pt0[i][0];
				pk(1,0) = Str[k][i][1] + Pt0[i][1];
				pk(2,0) = Str[k][i][2] + Pt0[i][2];

				arma::mat pkX = crossProductMatrix(pk);

				H.submat(c,0,c+2,2) = pkX*(ord*dt);
				H.submat(c,3,c+2,5) = pkX*(ord*dt)*(ord*dt)*0.5;
				H.submat(c,6,c+2,8).eye();
				H.submat(c,9,c+2,11).eye();

				H.submat(c,6,c+2,8)*=(ord*dt);
				H.submat(c,9,c+2,11)*=(ord*dt)*(ord*dt)*0.5;

			}
	}

	arma::mat X = -(H.t()*H).i()*H.t()*Pm;

	arma::mat q = X.submat(0,0,2,0)*dt + X.submat(3,0,5,0)*dt*dt/2;

	arma::mat Q = crossProductMatrix(q);

	arma::mat C1 = crossProductMatrix(X.submat(0,0,2,0))*dt;
	arma::mat C2 = crossProductMatrix(X.submat(3,0,5,0))*dt*dt*0.5;

	arma::mat I;
	I.eye(3,3);

	arma::mat C = (I + C1 + C2).i();

	arma::mat a1 = C*X.submat(6,0,8,0);
	arma::mat a2 = C*X.submat(9,0,11,0);

	rotMat = CRP2ROT(q);
	transT = a1*dt + a2*dt*dt/2;


	return true;

}


bool ppTransEst::solveLinear2ndOrderWtCov(std::vector<cv::Vec3f> Pt0,
		std::vector<cv::Vec3f> Pt1, std::vector<cv::Vec3f> Pt2,
		std::vector<arma::mat> covPt0, std::vector<arma::mat> covPt1,
		std::vector<arma::mat> covPt2, arma::mat& matA, arma::mat& matB,
		arma::mat& sigmaA, arma::mat& sigmaB,
		std::vector<int>& ransacEjectID) {


	double dt = 0.1;
	nP = Pt2.size();

	Pm.zeros(nP*3*2,1);
	arma::mat H(nP*3*2,12);
	H.zeros();

	for(int i =0; i < nP; i++)
	{
		int c = 3*i;

		Pm(c,0) = Pt1[i][0] - Pt2[i][0];
		Pm(c+1,0) = Pt1[i][1] - Pt2[i][1];
		Pm(c+2,0) = Pt1[i][2] - Pt2[i][2];

		arma::mat pk(3,1);

		pk(0,0) = Pt2[i][0] + Pt1[i][0];
		pk(1,0) = Pt2[i][1] + Pt1[i][1];
		pk(2,0) = Pt2[i][2] + Pt1[i][2];

		arma::mat pkX = crossProductMatrix(pk);

		H.submat(c,0,c+2,2) = pkX;
		H.submat(c,3,c+2,5) = pkX*dt;
		H.submat(c,6,c+2,8).eye();
		H.submat(c,9,c+2,11).eye();

		H.submat(c,9,c+2,11)*=dt;
	}

	for(int i =0; i < nP; i++)
	{
		int c = 3*i + 3*nP;

		Pm(c,0) = Pt0[i][0] - Pt2[i][0];
		Pm(c+1,0) = Pt0[i][1] - Pt2[i][1];
		Pm(c+2,0) = Pt0[i][2] - Pt2[i][2];

		arma::mat pk(3,1);

		pk(0,0) = Pt2[i][0] + Pt0[i][0];
		pk(1,0) = Pt2[i][1] + Pt0[i][1];
		pk(2,0) = Pt2[i][2] + Pt0[i][2];

		arma::mat pkX = crossProductMatrix(pk);

		H.submat(c,0,c+2,2) = pkX;
		H.submat(c,3,c+2,5) = 2*pkX*dt;

		H.submat(c,6,c+2,8).eye();
		H.submat(c,9,c+2,11).eye();

		H.submat(c,9,c+2,11)*=2*dt;
	}

	arma::mat X = (H.t()*H).i()*H.t()*Pm;

	arma::mat q = X.submat(0,0,2,0) + X.submat(3,0,5,0)*dt;

	arma::mat Q = crossProductMatrix(q);

	arma::mat C1 = crossProductMatrix(X.submat(0,0,2,0));
	arma::mat C2 = crossProductMatrix(X.submat(3,0,5,0))*dt;

	arma::mat I;
	I.eye(3,3);

	arma::mat C = (I + C1 + C2).i();

	arma::mat a1 = C*X.submat(6,0,8,0);
	arma::mat a2 = C*X.submat(9,0,11,0);


	matA.zeros(3,2);
	matB.zeros(3,2);

	matA.col(0) = a1.col(0);
	matA.col(1) = a2.col(0);

	matB.col(0) = X.submat(0,0,2,0);
	matB.col(1) = X.submat(3,0,5,0);

	std::vector<std::vector<cv::Vec3f>> vctVctPt;
	std::vector<std::vector<arma::mat>> vctVctCovPt;

	vctVctPt.push_back(Pt0);
	vctVctPt.push_back(Pt1);
	vctVctPt.push_back(Pt2);

	vctVctCovPt.push_back(covPt0);
	vctVctCovPt.push_back(covPt1);
	vctVctCovPt.push_back(covPt2);

	kinCov.update(H,vctVctPt,vctVctCovPt,X,dt,sigmaA,sigmaB);

	return true;

}


bool ppTransEst::solveLinear3rdOrder(std::vector<cv::Vec3f> Pt0,
		std::vector<cv::Vec3f> Pt1, std::vector<cv::Vec3f> Pt2,
		std::vector<cv::Vec3f> Pt3, arma::mat& rotMat, arma::mat& transT,
		std::vector<int>& ransacEjectID) {


	double dt = 0.1;
	nP = Pt2.size();

	Pm.zeros(nP*3*3,1);
	arma::mat H(nP*3*3,18);
	H.zeros();

	std::vector<cv::Vec3f> Str[3];
	Str[0] = Pt1;
	Str[1] = Pt2;
	Str[2] = Pt3;

	for(int k = 0; k < 3; k++)
	{
			for(int i =0; i < nP; i++)
			{
				int c = 3*i + k*3*nP;

				int ord = k+1;

				Pm(c,0) = Pt0[i][0] - Str[k][i][0];
				Pm(c+1,0) = Pt0[i][1] - Str[k][i][1];
				Pm(c+2,0) = Pt0[i][2] - Str[k][i][2];

				arma::mat pk(3,1);

				pk(0,0) = Str[k][i][0] + Pt0[i][0];
				pk(1,0) = Str[k][i][1] + Pt0[i][1];
				pk(2,0) = Str[k][i][2] + Pt0[i][2];

				arma::mat pkX = crossProductMatrix(pk);

				H.submat(c,0,c+2,2) = pkX*(ord*dt);
				H.submat(c,3,c+2,5) = pkX*(ord*dt)*(ord*dt)*0.5;
				H.submat(c,6,c+2,8) = pkX*(ord*dt)*(ord*dt)*(ord*dt)/6.;
				H.submat(c,9,c+2,11).eye();
				H.submat(c,12,c+2,14).eye();
				H.submat(c,15,c+2,17).eye();

				H.submat(c,9,c+2,11)*=(ord*dt);
				H.submat(c,12,c+2,14)*=(ord*dt)*(ord*dt)*0.5;
				H.submat(c,15,c+2,17)*=(ord*dt)*(ord*dt)*(ord*dt)/6.;

			}
	}

	arma::mat X = -(H.t()*H).i()*H.t()*Pm;

//	arma::mat q = X.submat(0,0,2,0) + X.submat(3,0,5,0)*dt + X.submat(6,0,8,0)*dt*dt*0.5;
//
//	arma::mat Q = crossProductMatrix(q);
//
//	arma::mat C1 = crossProductMatrix(X.submat(0,0,2,0));
//	arma::mat C2 = crossProductMatrix(X.submat(3,0,5,0))*dt;
//	arma::mat C3 = crossProductMatrix(X.submat(6,0,8,0))*dt*dt*0.5;
//
//	arma::mat I;
//	I.eye(3,3);
//
//	arma::mat C = (I + C1 + C2 + C3).i();
//
//	arma::mat a1 = C*X.submat(9,0,11,0);
//	arma::mat a2 = C*X.submat(12,0,14,0);
//	arma::mat a3 = C*X.submat(15,0,17,0);
//
//	rotMat = CRP2ROT(q);
//	transT = a1 + a2*dt + a3*dt*dt*0.5;


	/// MkII
	arma::mat q = X.submat(0,0,2,0)*dt + X.submat(3,0,5,0)*dt*dt/2 + X.submat(6,0,8,0)*dt*dt*dt/6;

	arma::mat Q = crossProductMatrix(q);

	arma::mat C1 = crossProductMatrix(X.submat(0,0,2,0))*dt;
	arma::mat C2 = crossProductMatrix(X.submat(3,0,5,0))*dt*dt*0.5;
	arma::mat C3 = crossProductMatrix(X.submat(6,0,8,0))*dt*dt*dt/6.;

	arma::mat I;
	I.eye(3,3);

	arma::mat C = (I + C1 + C2 + C3).i();

	arma::mat a1 = C*X.submat(9,0,11,0);
	arma::mat a2 = C*X.submat(12,0,14,0);
	arma::mat a3 = C*X.submat(15,0,17,0);

	rotMat = CRP2ROT(q);
	transT = a1*dt + a2*dt*dt/2 + a3*dt*dt*dt/6;


	return true;

}

bool ppTransEst::solveLinear3rdOrderWtCov(std::vector<cv::Vec3f> Pt0,
		std::vector<cv::Vec3f> Pt1, std::vector<cv::Vec3f> Pt2,
		std::vector<cv::Vec3f> Pt3, std::vector<arma::mat> covPt0,
		std::vector<arma::mat> covPt1, std::vector<arma::mat> covPt2,
		std::vector<arma::mat> covPt3, arma::mat& matA, arma::mat& matB,
		arma::mat& sigmaA, arma::mat& sigmaB,
		std::vector<int>& ransacEjectID) {



	double dt = 0.1;
	nP = Pt2.size();

	Pm.zeros(nP*3*3,1);
	arma::mat H(nP*3*3,18);
	H.zeros();

	std::vector<cv::Vec3f> Str[3];
	Str[0] = Pt1;
	Str[1] = Pt2;
	Str[2] = Pt3;

	std::vector<arma::mat> covStr[3];
	covStr[0] = covPt1;
	covStr[1] = covPt2;
	covStr[2] = covPt3;

	arma::mat W = arma::zeros(nP*3*3,nP*3*3);

	for(int k = 0; k < 3; k++)
	{
			for(int i =0; i < nP; i++)
			{
				int c = 3*i + k*3*nP;

				int ord = k+1;

				Pm(c,0) =   Pt0[i][0] - Str[k][i][0];
				Pm(c+1,0) = Pt0[i][1] - Str[k][i][1];
				Pm(c+2,0) = Pt0[i][2] - Str[k][i][2];

				W(c,c) = 1/sqrt(covPt0[i](0,0)*covPt0[i](0,0) + covStr[k][i](0,0)*covStr[k][i](0,0));
				W(c+1,c+1) = 1/sqrt(covPt0[i](1,1)*covPt0[i](1,1) + covStr[k][i](1,1)*covStr[k][i](1,1));
				W(c+2,c+2) = 1/sqrt(covPt0[i](2,2)*covPt0[i](2,2) + covStr[k][i](2,2)*covStr[k][i](2,2));

				arma::mat pk(3,1);

				pk(0,0) = Str[k][i][0] + Pt0[i][0];
				pk(1,0) = Str[k][i][1] + Pt0[i][1];
				pk(2,0) = Str[k][i][2] + Pt0[i][2];

				arma::mat pkX = crossProductMatrix(pk);

				H.submat(c,0,c+2,2) = pkX;
				H.submat(c,3,c+2,5) = pkX*(ord*dt);
				H.submat(c,6,c+2,8) = pkX*(ord*dt)*(ord*dt)*0.5;
				H.submat(c,9,c+2,11).eye();
				H.submat(c,12,c+2,14).eye();
				H.submat(c,15,c+2,17).eye();

				H.submat(c,12,c+2,14)*=(ord*dt);
				H.submat(c,15,c+2,17)*=(ord*dt)*(ord*dt)*0.5;

			}
	}

	arma::mat X = (H.t()*W*H).i()*H.t()*W*Pm;

	arma::mat q = X.submat(0,0,2,0) + X.submat(3,0,5,0)*dt + X.submat(6,0,8,0)*dt*dt*0.5;

	arma::mat Q = crossProductMatrix(q);

	arma::mat C1 = crossProductMatrix(X.submat(0,0,2,0));
	arma::mat C2 = crossProductMatrix(X.submat(3,0,5,0))*dt;
	arma::mat C3 = crossProductMatrix(X.submat(6,0,8,0))*dt*dt*0.5;

	arma::mat I;
	I.eye(3,3);

	arma::mat C = (I + C1 + C2 + C3).i();

	arma::mat a1 = C*X.submat(9,0,11,0);
	arma::mat a2 = C*X.submat(12,0,14,0);
	arma::mat a3 = C*X.submat(15,0,17,0);

//	rotMat = CRP2ROT(q);
//	transT = a1 + a2*dt + a3*dt*dt*0.5;


	matA.zeros(3,3);
	matB.zeros(3,3);

	matA.col(0) = a1.col(0);
	matA.col(1) = a2.col(0);
	matA.col(2) = a3.col(0);

	matB.col(0) = X.submat(0,0,2,0);
	matB.col(1) = X.submat(3,0,5,0);
	matB.col(2) = X.submat(6,0,8,0);

	std::vector<std::vector<cv::Vec3f>> vctVctPt;
	std::vector<std::vector<arma::mat>> vctVctCovPt;

	vctVctPt.push_back(Pt0);
	vctVctPt.push_back(Pt1);
	vctVctPt.push_back(Pt2);
	vctVctPt.push_back(Pt3);

	vctVctCovPt.push_back(covPt0);
	vctVctCovPt.push_back(covPt1);
	vctVctCovPt.push_back(covPt2);
	vctVctCovPt.push_back(covPt3);

	kinCov.update(H,vctVctPt,vctVctCovPt,X,dt,sigmaA,sigmaB);

	return true;


}

bool ppTransEst::solveLinearWtCov(std::vector<cv::Vec3f> Pt0,
		std::vector<cv::Vec3f> Pt1, std::vector<arma::mat> covPt0,
		std::vector<arma::mat> covPt1, arma::mat& matA, arma::mat& matB,
		arma::mat& sigmaA, arma::mat& sigmaB,
		std::vector<int>& ransacEjectID) {

	Pk.zeros(3*Pt0.size(),1);
	Pm.zeros(3*Pt0.size(),1);
	nP = Pt0.size();

	arma::mat H(Pt0.size()*3,6);

	arma::mat W(Pt0.size()*3,Pt0.size()*3);
	W.eye();

	for(int i =0; i < nP; i++)
	{
		int c = 3*i;

		Pm(c,0) = Pt0[i][0] - Pt1[i][0];
		Pm(c+1,0) = Pt0[i][1] - Pt1[i][1];
		Pm(c+2,0) = Pt0[i][2] - Pt1[i][2];

		arma::mat pk(3,1);

		pk(0,0) = Pt1[i][0] + Pt0[i][0];
		pk(1,0) = Pt1[i][1] + Pt0[i][1];
		pk(2,0) = Pt1[i][2] + Pt0[i][2];

		arma::mat pkX = crossProductMatrix(pk);

		H.submat(c,0,c+2,2) = pkX;
		H.submat(c,3,c+2,5).eye();

		W.submat(c,c,c+2,c+2) = covPt0[i];

	}

	arma::mat X = (H.t()*H).i()*H.t()*Pm;

	arma::mat q = X.submat(0,0,2,0);

	arma::mat Q = crossProductMatrix(q);

	arma::mat I;
	I.eye(3,3);

	arma::mat t = (I + Q).i()*X.submat(3,0,5,0);

	arma::mat Y = H*X;
	arma::mat dY = Pm - Y;

	arma::mat var = arma::var(dY);

	bool flg = true;

	double sig = sqrt(var(0,0));

	for(int i=0; i < dY.n_rows; i++)
	{
		if(fabs(dY(i,0)) > 6*sig)
		{
			ransacEjectID.push_back(i);
			flg = false;
		}
	}

//	rotMat = CRP2ROT(q);
//	transT = t;

	matA = t;
	matB = q;

	std::vector<std::vector<cv::Vec3f>> vctVctPt;
	std::vector<std::vector<arma::mat>> vctVctCovPt;

	vctVctPt.push_back(Pt0);
	vctVctPt.push_back(Pt1);
	vctVctCovPt.push_back(covPt0);
	vctVctCovPt.push_back(covPt1);

	double dt = 0.1;
	kinCov.update(H,vctVctPt,vctVctCovPt,X,dt,sigmaA,sigmaB);

	return flg;
}
