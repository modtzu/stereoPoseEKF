/*
 * plotData.h
 *
 *  Created on: Oct 2, 2015
 *      Author: xwong
 */

#ifndef PLOTDATA_H_
#define PLOTDATA_H_

#include "armadillo"
#include "gnuplot-iostream.h"

#define PL_HOLD_ON 0x01
#define PL_GRID_ON 0x02
#define PL_LABEL_ON 0x04


class plotData {
private:
	  Gnuplot pl;

	  bool holdFlg;

	  int holdID;

public:

	plotData();

	template <typename T>
	void init(T rngMin, T rngMax);

	template <typename T>
	void plot(std::vector<T> x, int plotID, std::string legend="", int Argb = 0x00)
	{
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

				pl << "plot ";

				if(hold)
					holdFlg = true;
			}
			else
				if(!hold)
					holdFlg = false;

			std::vector<T> vctNum;

			for(int i = 0; i<x.size(); i++)
			{
				vctNum.push_back(i);
			}

			std::pair<std::vector<T>,std::vector<T> >  par = std::make_pair(vctNum,x);

			pl << pl.binFile1d(par, "record") << "with lines title ' "<<legend<<" '";

			if(holdFlg)
				pl<<" , ";
			else
			{
				pl<<std::endl;
			}

	}

	template <typename T>
	void plot(std::vector<T> x, std::vector<T> y, int plotID, std::string legend="", int Argb = 0x00)
	{
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

				pl <<"set xlabel \"time(sec)\" \n";
						pl <<"show xlabel \n";
						pl <<"set ylabel \"dr(km)\" \n";
						pl <<"show ylabel \n";

				pl << "plot ";

				if(hold)
					holdFlg = true;
			}
			else
				if(!hold)
					holdFlg = false;

			std::pair<std::vector<T>,std::vector<T> >  par = std::make_pair(x,y);

			pl << pl.binFile1d(par, "record") << "with lines title ' "<<legend<<" '";

			if(holdFlg)
				pl<<" , ";
			else
			{
				pl<<std::endl;
			}

	}

	void plot(arma::mat data, int plotID, std::string legend="", int Argb = 0x00);

	void setHold(bool state, int holdID);

	template <typename T>
	void plot3(std::vector<T> x, std::vector<T> y, std::vector<T> z, int plotID = 0, std::string legend="", int Argb = 0x00)
	 {
		bool hold = false, grid = false, label = false;

		if((Argb & PL_HOLD_ON))
			hold = true;

		if((Argb & PL_GRID_ON))
			grid = true;

		if((Argb & PL_LABEL_ON)== PL_LABEL_ON)
			label = true;

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
					pl << "set grid ztics";
					pl<<std::endl;
				}

//			if(label)
			{
					pl <<"set xlabel \"x(km)\" \n";
					pl <<"show xlabel \n";
					pl <<"set ylabel \"y(km)\" \n";
					pl <<"show ylabel \n";
					pl <<"set zlabel \"z(km)\" \n";
					pl <<"show zlabel \n";
			}

			pl << "splot ";

			if(hold)
				holdFlg = true;
		}
		else
			if(!hold)
				holdFlg = false;

		std::pair< std::pair<std::vector<T>,std::vector<T> >, std::vector<T> > par = std::make_pair(std::make_pair(x,y),z);

		pl << pl.binFile1d(par, "record") << "with lines title ' "<<legend<<" '";

		if(holdFlg)
			pl<<" , ";
		else
		{
			pl<<std::endl;
		}

	 }

	void save(char* fileName, int sx = 1400, int sy = 900, int fontSize = 20);

	virtual ~plotData();
};

#endif /* PLOTDATA_H_ */
