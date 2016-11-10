/*
 * stereoSolver.cpp
 *
 *  Created on: Sep 9, 2016
 *      Author: xwong
 */

#include "stereoSolver.h"

stereoSolver::stereoSolver() {
	// TODO Auto-generated constructor stub
	 u0 = 0; // image principle point x
	 v0 = 0; // image principle point y
	 f = 0;  // focus length
	 B = 0;  // baseline distance
}

stereoSolver::~stereoSolver() {
	// TODO Auto-generated destructor stub
}

bool stereoSolver::computeDepth(std::vector<cv::KeyPoint>* ptrFt0,
		std::vector<cv::KeyPoint>* ptrFt1, std::vector<arma::mat>* ptrCovFt0,
		std::vector<arma::mat>* ptrCovFt1, std::vector<cv::Vec3f>& vctPt) {


	if(u0 == 0 || v0 == 0)
	{
		std::cout<<"Error : Did not specify principle points\n";
		return false;
	}

	if(f == 0 )
	{
		std::cout<<"Error : Did not specify camera focal length\n";
		return false;
	}

	if(B == 0 )
	{
		std::cout<<"Error : Did not specify base line length\n";
		return false;
	}

	if(ptrFt0->size()!=ptrFt1->size())
	{
		std::cout<<"Error : Ft0 size != Ft1 Size\n";
		return false;
	}

	arma::mat dispMat(ptrFt0->size(),1);

	vctPt.clear();

	for(unsigned int i= 0; i < ptrFt0->size(); i++)
	{
		float D = fabs((*ptrFt0)[i].pt.x-(*ptrFt1)[i].pt.x);

		double d = (B*f)/D;

		float wl = (*ptrCovFt0)[i](0,0)*(*ptrCovFt0)[i](0,0) + (*ptrCovFt0)[i](1,1)*(*ptrCovFt0)[i](1,1);
		float wr = (*ptrCovFt1)[i](0,0)*(*ptrCovFt1)[i](0,0) + (*ptrCovFt1)[i](1,1)*(*ptrCovFt1)[i](1,1);

		wl = 1/sqrt(wl);
		wr = 1/sqrt(wr);

		float wt = wl + wr;

			float z = d;
			float xl = ( (*ptrFt0)[i].pt.x -u0);//*B/D;
			float yl = ( (*ptrFt0)[i].pt.y -v0);//*B/D;

			float xr = ( (*ptrFt1)[i].pt.x -u0);//*B/D;
			float yr = ( (*ptrFt1)[i].pt.y -v0);//*B/D;

			float x = (wl*xl + wr*xr + wr*d)*B/(wt*D);
			float y = (wl*yl + wr*yr)*B/(wt*D);

			vctPt.push_back(cv::Vec3f(x,y,z));
	}

	return true;
}

bool stereoSolver::computeDepth(std::vector<cv::KeyPoint>* ptrFt0,
		std::vector<cv::KeyPoint>* ptrFt1, std::vector<cv::Vec3f>& vctPt) {

	if(u0 == 0 || v0 == 0)
	{
		std::cout<<"Error : Did not specify principle points\n";
		return false;
	}

	if(f == 0 )
	{
		std::cout<<"Error : Did not specify camera focal length\n";
		return false;
	}

	if(B == 0 )
	{
		std::cout<<"Error : Did not specify base line length\n";
		return false;
	}

	if(ptrFt0->size()!=ptrFt1->size())
	{
		std::cout<<"Error : Ft0 size != Ft1 Size\n";
		return false;
	}

	arma::mat dispMat(ptrFt0->size(),1);

	vctPt.clear();

	for(unsigned int i= 0; i < ptrFt0->size(); i++)
	{
		float D = fabs((*ptrFt0)[i].pt.x-(*ptrFt1)[i].pt.x);

		double d = (B*f)/D;

			float z = d;
			float xl = ( (*ptrFt0)[i].pt.x -u0);//*B/D;
			float yl = ( (*ptrFt0)[i].pt.y -v0);//*B/D;

			float xr = ( (*ptrFt1)[i].pt.x -u0);//*B/D;
			float yr = ( (*ptrFt1)[i].pt.y -v0);//*B/D;

			float x = 0.5*(xl + xr + d)*B/D;
			float y = 0.5*(yl + yr)*B/D;

			vctPt.push_back(cv::Vec3f(x,y,z));
	}

	return true;
}

bool stereoSolver::computeDepthWtCov(std::vector<cv::KeyPoint>* ptrFt0,
		std::vector<cv::KeyPoint>* ptrFt1, std::vector<cv::Vec3f>& vctPt,
		std::vector<arma::mat>* ptrCovFt0, std::vector<arma::mat>* ptrCovFt1,
		std::vector<arma::mat>& vctCovPt) {

	vctCovPt.clear();

	std::vector<cv::Vec3f> vctPt1;

	if(!computeDepth(ptrFt0,ptrFt1,vctPt))
		return false;

//	if(!computeDepth(ptrFt0, ptrFt1,
//			ptrCovFt0, ptrCovFt1, vctPt))
//				return false;

	int numOfFt = ptrFt0->size();

	for(int i =0 ;i < numOfFt; i++)
	{
		float ul = (*ptrFt0)[i].pt.x;
		float vl = (*ptrFt0)[i].pt.y;
		float ur = (*ptrFt1)[i].pt.x;
		float vr = (*ptrFt1)[i].pt.y;

//		float wl = (*ptrCovFt0)[i](0,0)*(*ptrCovFt0)[i](0,0) + (*ptrCovFt0)[i](1,1)*(*ptrCovFt0)[i](1,1);
//		float wr = (*ptrCovFt1)[i](0,0)*(*ptrCovFt1)[i](0,0) + (*ptrCovFt1)[i](1,1)*(*ptrCovFt1)[i](1,1);
//
//		wl = 1/wl;
//		wr = 1/wr;
//
//		float wt = wl + wr;

		// weight
		float wl = 0.5;
		float wr = 0.5;
		float wt = wl + wr;

		double D2 = (ul-ur)*(ul-ur);
		double d = fabs(ul - ur);

		float dDdUl = (ul - ur)/d;
		float dDdVl = (vl - vr)/d;

		float dDdUr = -dDdUl;
		float dDdVr = -dDdVl;

		arma::mat H(3,4);

		/// dX
		H(0,0) = -B/(D2*wt)*dDdUl*(wl*ul + wr*ur) + B/(d*wt)*wl;
		H(0,1) = -B/(D2*wt)*dDdVl*(wl*ul + wr*ur);
		H(0,2) = -B/(D2*wt)*dDdUr*(wl*ul + wr*ur) + B/(d*wt)*wr;
		H(0,3) = -B/(D2*wt)*dDdVr*(wl*ul + wr*ur);

		/// dY
		H(1,0) = -B/(D2*wt)*dDdUl*(wl*vl + wr*vr);
		H(1,1) = -B/(D2*wt)*dDdVl*(wl*vl + wr*vr) + B/(d*wt)*wl;
		H(1,2) = -B/(D2*wt)*dDdUr*(wl*vl + wr*vr);
		H(1,3) = -B/(D2*wt)*dDdVr*(wl*vl + wr*vr) + B/(d*wt)*wr;

		/// dZ
		H(2,0) = dDdUl;
		H(2,1) = dDdVl;
		H(2,2) = dDdUr;
		H(2,3) = dDdVl;
		H.row(2) = -f*B/D2*H.row(2);

		arma::mat SigmaUV(4,4);

		SigmaUV.submat(0,0,1,1) = (*ptrCovFt0)[i];
		SigmaUV.submat(2,2,3,3) = (*ptrCovFt1)[i];

		arma::mat CovP = H*SigmaUV*H.t();
		vctCovPt.push_back(CovP);
	}

	return true;
}
