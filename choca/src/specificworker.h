/*
 *    Copyright (C)2019 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
       \brief
       @author authorname
*/



#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <list>
#include <genericworker.h>
#include <innermodel/innermodel.h>
#include "floormeter.h"
using namespace std;

struct mycoordenada{
	int x;
	int y;
	float angulo;//angulo en radianes a donde mira el robor
	int direccion;//0-7 posiciones del robot segun el cuadrante  de la circurferencia
};


class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(TuplePrx tprx);
	//SpecificWorker(MapPrx& mprx);

	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);


public slots:
	void compute();
	void initialize(int period);
	void giroRandom(float rot);
	void giroNormal(float rot);
//	void resetSlot();
	void anadirLista(float rot);

private:
	
	std::shared_ptr<InnerModel> innerModel;
//    FloorMeter fm;

	std::list<mycoordenada> lista;


};

#endif
