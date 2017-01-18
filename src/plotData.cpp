/*
 * plotData.cpp
 *
 *  Created on: Oct 2, 2015
 *      Author: xwong
 */

#include "plotData.h"

plotData::plotData() {
	// TODO Auto-generated constructor stub
	holdFlg = false;
	holdID= -1;
}

plotData::~plotData() {
	// TODO Auto-generated destructor stub
//	delete pl;
}


void plotData::save(char* fileName, int sx, int sy, int fontSize)
{
	pl<<"set terminal png"<<" size "<<sx<<","<<sy<<" enhanced font 'Verdana,"<<fontSize<<"'";
	pl << std::endl;

	pl<<"set output \" "<<fileName<<"\" ";
	pl << std::endl;

	pl<<"replot";
	pl << std::endl;

	pl<<"set term x11";
	pl<<std::endl;
}

void plotData::plot(arma::mat data, int plotID = 0, std::string legend, int Argb) {

	bool hold = false, grid = false;

	if((Argb & PL_HOLD_ON) == PL_HOLD_ON)
			hold = true;

		if((Argb & PL_GRID_ON) == PL_GRID_ON)
			grid = true;

		if(!holdFlg)
		{
			pl <<"set term X11 "<<plotID;
			pl << std::endl;

			if(grid)
				{
					pl << "set grid xtics";
					pl<<std::endl;
					pl << "set grid ytics";
					pl<<std::endl;
				}

			pl <<"set xlabel \"Frame\" \n";
					pl <<"show xlabel \n";
					pl <<"set ylabel \"error(q)\" \n";
					pl <<"show ylabel \n";

			pl << "plot ";

			if(hold)
				holdFlg = true;
		}
		else
			if(!hold)
				holdFlg = false;

	if(data.n_cols == 1)
	{
		arma::mat data2(data.n_rows,2);
		for(int i =0 ; i < data.n_rows; i++)
		{
			data2(i,0) = i;
		}
		data2.col(1) = data.col(0);
		data = data2;
	}

	pl << pl.binFile1d(data, "record") << "with lines title ' "<<legend<<" '";

	if(!holdFlg)
		pl << std::endl;
	else
		pl <<" , ";

}

void plotData::setHold(bool state, int holdID) {

	switch(state)
	{
	case 0: /// off
		holdFlg = false;
		pl << std::endl;

		break;
	case 1: /// on
		holdFlg = true;

		pl <<"set term X11 "<<holdID;
		pl << std::endl;
		pl<<"plot ";

		break;
	}

}
