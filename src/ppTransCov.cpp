/*
 * ppTransCov.cpp
 *
 *  Created on: Oct 14, 2016
 *      Author: xwong
 */

#include "ppTransCov.h"

ppTransCov::ppTransCov() {
	// TODO Auto-generated constructor stub
	dRhodPx = arma::zeros(3,3);
	dRhodPy = arma::zeros(3,3);
	dRhodPz = arma::zeros(3,3);

	dRhodPx(1,2) = -1;
	dRhodPx(2,1) = 1;

	dRhodPy(0,2) = 1;
	dRhodPy(2,0) = -1;

	dRhodPz(0,1) = -1;
	dRhodPz(1,0) = 1;

}

bool ppTransCov::update(arma::mat matH, std::vector< std::vector<cv::Vec3f> > vctVctPt, std::vector< std::vector<arma::mat> > vctVctCovPt,arma::mat x,double dt, arma::mat& sigmaA, arma::mat& sigmaB ) {

	int n = vctVctPt[0].size();
	int numOfSt = vctVctCovPt.size();

	int numOfP = numOfSt*n*3;


	arma::mat dXdP = arma::zeros(6*(numOfSt-1),numOfP);

	for(int t = 0; t < vctVctPt.size(); t++)
	{
		int j = vctVctPt.size()-t-1;

		arma::mat dXdPt = compdXdP(matH, vctVctPt[j],x,dt,t,numOfSt);

		int k = 3*n*t;

		dXdP.submat(0,k,6*(numOfSt-1)-1,k+dXdPt.n_cols-1) = dXdPt;
	}

	arma::mat SigPt = arma::zeros(3*n*numOfSt,3*n*numOfSt);

	for(int t=0; t < vctVctPt.size(); t++)
	{
		int j = vctVctPt.size()-t-1;

		for(int i=0; i < n; i++)
		{
			int k= 3*i + t*3*n;

			SigPt.submat(k,k,k+2,k+2) = vctVctCovPt[j][i];
		}
	}

	arma::mat db1dp = arma::zeros(3,numOfP), db2dp = arma::zeros(3,numOfP), db3dp = arma::zeros(3,numOfP);
	arma::mat dd1dp = arma::zeros(3,numOfP), dd2dp = arma::zeros(3,numOfP), dd3dp = arma::zeros(3,numOfP);

	arma::mat b1 = arma::zeros(3,1), b2 = arma::zeros(3,1), b3 = arma::zeros(3,1);
	arma::mat d1 = arma::zeros(3,1), d2 = arma::zeros(3,1), d3 = arma::zeros(3,1);
	arma::mat a1 = arma::zeros(3,1), a2 = arma::zeros(3,1), a3 = arma::zeros(3,1);

	db1dp = dXdP.submat(0,0,2,numOfP-1);
	dd1dp = dXdP.submat(3,0,5,numOfP-1);

	b1 = x.submat(0,0,2,0);
	d1 = x.submat(3,0,5,0);

	if(vctVctPt.size()>2)
	{
		 db2dp = dXdP.submat(3,0,5,numOfP-1);

		 dd1dp = dXdP.submat(6,0,8,numOfP-1);
		 dd2dp = dXdP.submat(9,0,11,numOfP-1);

		 b2 = x.submat(3,0,5,0);

		 d1 = x.submat(6,0,8,0);
		 d2 = x.submat(9,0,11,0);
	}

	if(vctVctPt.size()>3)
	{
		db2dp = dXdP.submat(3,0,5,numOfP-1);
		db3dp = dXdP.submat(6,0,8,numOfP-1);


		dd1dp = dXdP.submat(9,0,11,numOfP-1);
		dd2dp = dXdP.submat(12,0,14,numOfP-1);
		dd3dp = dXdP.submat(15,0,17,numOfP-1);

		b2 = x.submat(3,0,5,0);
		b3 = x.submat(6,0,8,0);

		d1 = x.submat(9,0,11,0);
		d2 = x.submat(12,0,14,0);
		d3 = x.submat(15,0,17,0);
	}

	arma::mat B = arma::eye(3,3) + UT.crossProductMat(b1.col(0)) + UT.crossProductMat(b2.col(0))*dt + + UT.crossProductMat(b1.col(0))*dt*dt/2;
	arma::mat Bi = B.i();

	a1 = Bi*d1;

	if(vctVctPt.size()>2)
		a2 = Bi*d2;

	if(vctVctPt.size()>3)
		a3 = Bi*d3;


	arma::mat dB1dpA1= arma::zeros(3,db3dp.n_cols);
	arma::mat dB1dpA2= arma::zeros(3,db3dp.n_cols);
	arma::mat dB1dpA3= arma::zeros(3,db3dp.n_cols);

	////dB1dA


	dB1dpA1.row(0) = -db3dp.row(0)*a1(1,0) + db2dp.row(0)*a1(2,0);
	dB1dpA1.row(1) =  db3dp.row(0)*a1(0,0) - db1dp.row(0)*a1(2,0);
	dB1dpA1.row(2) = -db2dp.row(0)*a1(0,0) + db2dp.row(0)*a1(1,0);

	dB1dpA2.row(0) = -db3dp.row(0)*a2(1,0) + db2dp.row(0)*a2(2,0);
	dB1dpA2.row(1) =  db3dp.row(0)*a2(0,0) - db1dp.row(0)*a2(2,0);
	dB1dpA2.row(2) = -db2dp.row(0)*a2(0,0) + db2dp.row(0)*a2(1,0);

	dB1dpA3.row(0) = -db3dp.row(0)*a3(1,0) + db2dp.row(0)*a3(2,0);
	dB1dpA3.row(1) =  db3dp.row(0)*a3(0,0) - db1dp.row(0)*a3(2,0);
	dB1dpA3.row(2) = -db2dp.row(0)*a3(0,0) + db2dp.row(0)*a3(1,0);


	////dB2dA

	arma::mat dB2dpA1= arma::zeros(3,db3dp.n_cols);
	arma::mat dB2dpA2= arma::zeros(3,db3dp.n_cols);
	arma::mat dB2dpA3= arma::zeros(3,db3dp.n_cols);

	if(vctVctPt.size()> 2)
	{
		dB2dpA1.row(0) = -db3dp.row(1)*a1(1,0) + db2dp.row(1)*a1(2,0);
		dB2dpA1.row(1) =  db3dp.row(1)*a1(0,0) - db1dp.row(1)*a1(2,0);
		dB2dpA1.row(2) = -db2dp.row(1)*a1(0,0) + db2dp.row(1)*a1(1,0);

		dB2dpA2.row(0) = -db3dp.row(1)*a2(1,0) + db2dp.row(1)*a2(2,0);
		dB2dpA2.row(1) =  db3dp.row(1)*a2(0,0) - db1dp.row(1)*a2(2,0);
		dB2dpA2.row(2) = -db2dp.row(1)*a2(0,0) + db2dp.row(1)*a2(1,0);

		dB2dpA3.row(0) = -db3dp.row(1)*a3(1,0) + db2dp.row(1)*a3(2,0);
		dB2dpA3.row(1) =  db3dp.row(1)*a3(0,0) - db1dp.row(1)*a3(2,0);
		dB2dpA3.row(2) = -db2dp.row(1)*a3(0,0) + db2dp.row(1)*a3(1,0);
	}

	////dB3dA
	arma::mat dB3dpA1 = arma::zeros(3,db3dp.n_cols);
	arma::mat dB3dpA2 = arma::zeros(3,db3dp.n_cols);
	arma::mat dB3dpA3 = arma::zeros(3,db3dp.n_cols);

	if(vctVctPt.size()> 3)
	{
		dB3dpA1.row(0) = -db3dp.row(2)*a1(1,0) + db2dp.row(2)*a1(2,0);
		dB3dpA1.row(1) =  db3dp.row(2)*a1(0,0) - db1dp.row(2)*a1(2,0);
		dB3dpA1.row(2) = -db2dp.row(2)*a1(0,0) + db2dp.row(2)*a1(1,0);

		dB3dpA2.row(0) = -db3dp.row(2)*a2(1,0) + db2dp.row(2)*a2(2,0);
		dB3dpA2.row(1) =  db3dp.row(2)*a2(0,0) - db1dp.row(2)*a2(2,0);
		dB3dpA2.row(2) = -db2dp.row(2)*a2(0,0) + db2dp.row(2)*a2(1,0);

		dB3dpA3.row(0) = -db3dp.row(2)*a3(1,0) + db2dp.row(2)*a3(2,0);
		dB3dpA3.row(1) =  db3dp.row(2)*a3(0,0) - db1dp.row(2)*a3(2,0);
		dB3dpA3.row(2) = -db2dp.row(2)*a3(0,0) + db2dp.row(2)*a3(1,0);
	}


	arma::mat dBdpA1 = dB1dpA1 + dB2dpA1*dt + dB3dpA1*dt*dt/2;
	arma::mat dBdpA2 = dB1dpA2 + dB2dpA2*dt + dB3dpA2*dt*dt/2;
	arma::mat dBdpA3 = dB1dpA3 + dB2dpA3*dt + dB3dpA3*dt*dt/2;

	arma::mat BtBiBt = (B.t()*B).i()*B.t();

	arma::mat dadp;
	arma::mat dbdp;

	if(vctVctCovPt.size()> 3)
	{
		dadp = arma::zeros(9,numOfP);
		dbdp = arma::zeros(9,numOfP);

		dadp.submat(0,0,2,numOfP-1) = BtBiBt*(dd1dp - dBdpA1);
		dadp.submat(3,0,5,numOfP-1) = BtBiBt*(dd2dp - dBdpA2);
		dadp.submat(6,0,8,numOfP-1) = BtBiBt*(dd3dp - dBdpA3);

		dbdp.submat(0,0,2,numOfP-1) = db1dp;
		dbdp.submat(3,0,5,numOfP-1) = db2dp;
		dbdp.submat(6,0,8,numOfP-1) = db3dp;

	}
	else if(vctVctCovPt.size()> 2)
	{
		dadp = arma::zeros(6,numOfP);
		dbdp = arma::zeros(6,numOfP);

		dadp.submat(0,0,2,numOfP-1) = BtBiBt*(dd1dp - dBdpA1);
		dadp.submat(3,0,5,numOfP-1) = BtBiBt*(dd2dp - dBdpA2);

		dbdp.submat(0,0,2,numOfP-1) = db1dp;
		dbdp.submat(3,0,5,numOfP-1) = db2dp;

	}
	else
	{
		dadp = BtBiBt*(dd1dp - dBdpA1);
		dbdp = db1dp;
	}

	sigmaA = dadp*SigPt*dadp.t();

	sigmaB = dbdp*SigPt*dbdp.t();

	return true;
}

ppTransCov::~ppTransCov() {
	// TODO Auto-generated destructor stub
}

arma::mat ppTransCov::compdXdP(arma::mat H, std::vector<cv::Vec3f> vctPt,
		arma::mat x, double dt, int tIndex, int numOfSt) {


/// NEED A FIX ON HOW TO HANDLE CASE WHEN USING 1st and 2nd order model!!!!!!!!!!!!!!!
/// massy indexing.....



	/// Eta = P(k) - P(k-i)
	/// Rho = P(k-i) + P(k);

	int n = vctPt.size();


/// dEtadP

	arma::mat dEtadP(3*n*(numOfSt-1),3*n);
	dEtadP.zeros();

	if(tIndex != 0)
	{
		dEtadP.submat(3*(tIndex-1)*n,0,3*(tIndex)*n-1,3*n-1) = -arma::eye(3*n,3*n);
	}
	else
	{
		dEtadP.submat(0,0,3*n-1,3*n-1) = arma::eye(3*n,3*n);

		if(numOfSt>2)
			dEtadP.submat(3*n,0,6*n-1,3*n-1) = arma::eye(3*n,3*n);

		if(numOfSt>3)
			dEtadP.submat(6*n,0,9*n-1,3*n-1) = arma::eye(3*n,3*n);
	}

/// dHdPX
	arma::mat b1 = x.submat(0,0,2,0);

	arma::mat b2 = arma::zeros(3,1);
	if(numOfSt>2)
		b2 = x.submat(3,0,5,0);

	arma::mat b3 = arma::zeros(3,1);

	if(numOfSt>3)
		b3 = x.submat(6,0,8,0);

	arma::mat dHdPx = arma::zeros((numOfSt-1)*3*n,3*n);

	if(tIndex==0)
	{
		for(int j =1; j < numOfSt; j++)
		{
			arma::mat temp(3,3);

			temp.col(0) = dRhodPx*b1 + dRhodPx*b2*(j*dt) + dRhodPx*b3*(j*dt)*(j*dt)/2;
			temp.col(1) = dRhodPy*b1 + dRhodPy*b2*(j*dt) + dRhodPy*b3*(j*dt)*(j*dt)/2;
			temp.col(2) = dRhodPz*b1 + dRhodPz*b2*(j*dt) + dRhodPz*b3*(j*dt)*(j*dt)/2;

			arma::mat temp2 = arma::zeros(3*n,3*n);

			for(int i=0; i < n; i++)
			{
				int k=3*i;

				temp2.submat(k,k,k+2,k+2) = temp;
			}

			dHdPx.submat((j-1)*3*n,0,j*3*n-1,3*n-1) = temp2;

		}
	}
	else
	{
		int j = tIndex;

		arma::mat temp(3,3);

		temp.col(0) = dRhodPx*b1 + dRhodPx*b2*(j*dt) + dRhodPx*b3*(j*dt)*(j*dt)/2;
		temp.col(1) = dRhodPy*b1 + dRhodPy*b2*(j*dt) + dRhodPy*b3*(j*dt)*(j*dt)/2;
		temp.col(2) = dRhodPz*b1 + dRhodPz*b2*(j*dt) + dRhodPz*b3*(j*dt)*(j*dt)/2;

		arma::mat temp2 = arma::zeros(3*n,3*n);

		for(int i=0; i < n; i++)
		{
			int k=3*i;

			temp2.submat(k,k,k+2,k+2) = temp;
		}

		dHdPx.submat((j-1)*3*n,0,j*3*n-1,3*n-1) = temp2;

	}


	arma::mat dXdP = (H.t()*H).i()*H.t()*(dEtadP - dHdPx);


	return dXdP;
}

