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

#include <genericworker.h>
#include <innermodel/innermodel.h>



class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(TuplePrx tprx);
	~SpecificWorker();
	bool fin;
	bool setParams(RoboCompCommonBehavior::ParameterList params);
	int estado;
	int threshold;
	float rot;
	int esquivarObs;
	struct coordenada{
		float x;
		float z;
	};
	struct recta
	{
		float a;
		float b;
		float c;
		bool pertenece(float x, float y, float radio = 50)
		{
			return fabs((a*x+b*y+c)<=radio);
		}
	};
	coordenada destino;
	float distanciaEsquivar;
	recta rObjetivo;
	void RCISMousePicker_setPick(Pick myPick);

public slots:




	bool GotoPoint_atTarget();
	void GotoPoint_go(string nodo, float x, float y, float alpha);
	void GotoPoint_stop();
	void GotoPoint_turn(float speed);


	void compute();
	void initialize(int period);
	void readRobotState();
	void girarHaciaDestino();
	void irDestino();
	void esquivarObstaculo();
	void girarObstaculo();
	void rodearObstaculo();
	void crearrObjetivo();
	bool verDestino();
	bool pertenecePunto(float X, float Z);
	float distancia(coordenada origen, coordenada destino);
	void Fin();


private:

	std::shared_ptr<InnerModel> innerModel;
	
	RoboCompGenericBase::TBaseState bState;
	RoboCompLaser::TLaserData ldata;
};

#endif
