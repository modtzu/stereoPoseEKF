/*
 * poseEKF.cpp
 *
 *  Created on: Nov 8, 2016
 *      Author: xwong
 */

#include "poseEKF.h"

poseEKF::poseEKF() {
	// TODO Auto-generated constructor stub

}

bool poseEKF::propagate(arma::mat matA, arma::mat matB, arma::mat sigmaA, arma::mat sigmaB, double dt,arma::mat& propX, arma::mat& propSigmaX) {

	arma::mat t = arma::zeros(3,1);
	arma::mat q = arma::zeros(3,1);

	for(int i =0; i < matA.n_cols; i++)
	{
		t += matA.col(i)*pow(dt,i)/UT.factorial(i);
		q += matB.col(i)*pow(dt,i)/UT.factorial(i);
	}

	arma::mat R = ppT.CRP2ROT(q);

	arma::mat sigmaT;

	if(sigmaA.n_cols > 8)
	{
		sigmaT = sigmaA.submat(0,0,2,2) + sigmaA.submat(3,3,5,5)*dt*dt + sigmaA.submat(6,6,8,8)*pow(dt,4)/4
					+ sigmaA.submat(0,3,2,5)*dt + sigmaA.submat(0,6,2,8)*dt*dt + sigmaA.submat(3,6,5,8)*pow(dt,3);

	}
	else if(sigmaA.n_cols > 5)
	{
		sigmaT = sigmaA.submat(0,0,2,2) + sigmaA.submat(3,3,5,5)*dt*dt
					+ sigmaA.submat(0,3,2,5)*dt;
	}
	else
	{
		sigmaT = sigmaA;
	}

	arma::mat sigmaQ;

	if(sigmaB.n_cols > 8)
	{

		sigmaQ = sigmaB.submat(0,0,2,2) + sigmaB.submat(3,3,5,5)*dt*dt + sigmaB.submat(6,6,8,8)*pow(dt,4)/4
					+ sigmaB.submat(0,3,2,5)*dt + sigmaB.submat(0,6,2,8)*dt*dt + sigmaB.submat(3,6,5,8)*pow(dt,3);


	}
	else if(sigmaB.n_cols > 5)
	{
		sigmaQ = sigmaB.submat(0,0,2,2) + sigmaB.submat(3,3,5,5)*dt*dt
					+ sigmaB.submat(0,3,2,5)*dt;

	}
	else
	{
		sigmaQ = sigmaB;
	}

	propX = arma::zeros(6,1);
	propSigmaX = arma::zeros(6,6);

	propX.submat(0,0,2,0) = t;
	propX.submat(3,0,5,0) = q;

	propSigmaX.submat(0,0,2,2) = sigmaT;
	propSigmaX.submat(3,3,5,5) = sigmaQ;

	return true;
}

void poseEKF::update(arma::mat Xkp, arma::mat SigKp,
		std::vector<arma::mat> vctPk,std::vector<arma::mat> vctSigPk,
		std::vector<arma::mat> vctPkm1, arma::mat& Xk,
		arma::mat& SigXk) {

	arma::mat tp = Xkp.submat(0,0,2,0);
	arma::mat qp = Xkp.submat(3,0,5,0);

	arma::mat Rp = ppT.CRP2ROT(qp);

	arma::mat Pt = SigKp.submat(0,0,2,2);
	arma::mat Pq = SigKp.submat(3,3,5,5);

	arma::mat mY = arma::zeros(3*vctPk.size(),1), pY = arma::zeros(3*vctPk.size(),1);

	arma::mat qu = qp;
	arma::mat tu = tp;
	arma::mat Ru;
	arma::mat SigU = SigKp;
	arma::mat Xu  = Xkp;

	arma::mat Pm = arma::zeros(3*vctPk.size(),3*vctPk.size());

	for(int i =0 ; i < vctPk.size(); i++)
	{
		int k = 3*i;
		Pm.submat(k,k,k+2,k+2) = vctSigPk[i];
	}

	arma::mat H, Kk;

	for(int it = 0; it < 5; it++)
	{
		Ru = ppT.CRP2ROT(qu);

		for(int i =0 ; i < vctPk.size(); i++)
		{
			int k = 3*i;

			mY.submat(k,0,k+2,0) = vctPk[i].col(0);
			pY.submat(k,0,k+2,0) = Ru*vctPkm1[i].col(0) + tu;
		}

		H = jacobianMat(qu,vctPkm1);

		Kk = SigKp*H.t()*(H*SigKp*H.t() + Pm).i();

		Xu = Xkp + Kk*(mY - pY - H*(Xkp - Xu));

		if(arma::norm(Xu - Xkp)< 1e-3)
			break;

	}

	SigU = (arma::eye(6,6) - Kk*H)*SigKp;

	Xk = Xu;

}

arma::mat poseEKF::jacobianMat(arma::mat q, std::vector<arma::mat> vctPk) {

	arma::mat R = ppT.CRP2ROT(q);

	arma::mat dldq = -pow((1 + arma::dot(q.col(0),q.col(0))),-1.5)*q;

	double l = 1/sqrt(1 + arma::dot(q.col(0),q.col(0)));

	arma::mat dr11dq;
	dr11dq<<q(0)<<-q(1)<<-q(2);
	dr11dq = 2*dr11dq.t();

	arma::mat dr22dq;
	dr22dq<<-q(0)<<q(1)<<-q(2);
	dr22dq = 2*dr22dq.t();

	arma::mat dr33dq;
	dr33dq<<-q(0)<<-q(1)<<q(2);
	dr33dq = 2*dr33dq.t();

	arma::mat dR11dQ = dldq*R(0,0) + dr11dq*l;
	arma::mat dR22dQ = dldq*R(1,1) + dr22dq*l;
	arma::mat dR33dQ = dldq*R(2,2) + dr33dq*l;

	arma::mat dR12dQ, dR21dQ;
	dR12dQ<<q(1)<<q(0)<<1;
	dR21dQ<<q(1)<<q(0)<<-1;

	dR12dQ =  dldq*R(0,1) + 2*dR12dQ.t()*l;
	dR21dQ =  dldq*R(1,0) + 2*dR21dQ.t()*l;

	arma::mat dR13dQ, dR31dQ;
	dR13dQ<<q(2)<<-1<<q(0);
	dR31dQ<<q(2)<< 1<<q(0);

	arma::mat dR32dQ, dR23dQ;
	dR32dQ<<-1<<q(2)<<q(1);
	dR23dQ<< 1<<q(2)<<q(1);

	dR13dQ =  dldq*R(0,2) + 2*dR13dQ.t()*l;
	dR23dQ =  dldq*R(1,2) + 2*dR23dQ.t()*l;

	dR31dQ =  dldq*R(2,0) + 2*dR31dQ.t()*l;
	dR32dQ =  dldq*R(2,1) + 2*dR32dQ.t()*l;

	arma::mat H(3*vctPk.size(),6);

	for(int i = 0; i < vctPk.size(); i++)
	{
		arma::mat dRdqP(3,3);

		dRdqP.row(0) = dR11dQ.col(0).t()*vctPk[i](0,0) + dR12dQ.col(0).t()*vctPk[i](1,0) + dR13dQ.col(0).t()*vctPk[i](2,0);
		dRdqP.row(1) = dR21dQ.col(0).t()*vctPk[i](0,0) + dR22dQ.col(0).t()*vctPk[i](1,0) + dR23dQ.col(0).t()*vctPk[i](2,0);
		dRdqP.row(2) = dR31dQ.col(0).t()*vctPk[i](0,0) + dR32dQ.col(0).t()*vctPk[i](1,0) + dR33dQ.col(0).t()*vctPk[i](2,0);

		int k = 3*i;

		H.submat(k,0,k+2,2) = dRdqP;
		H.submat(k,3,k+2,5) = arma::eye(3,3);
	}

	return H;

}

poseEKF::~poseEKF() {
	// TODO Auto-generated destructor stub
}

