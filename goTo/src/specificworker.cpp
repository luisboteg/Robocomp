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
#include "specificworker.h"
#include <math.h>
const float pi=3.14159265358979323846;

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx) : GenericWorker(tprx)
{

}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	//       THE FOLLOWING IS JUST AN EXAMPLE

	try
	{
		innerModel = std::make_shared<InnerModel>( "/home/robocomp/robocomp/files/innermodel/mimapa.xml" );
		//timer.start(Period);
	}
	catch(std::exception e) { qFatal("Error reading config params"); }

	return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;
	this->Period = period;
	timer.start(Period);
	this->estado=9;
	this->threshold=200;
	this->rot = 0.6;
	this->fin=false;
}

void SpecificWorker::compute()
{ 
	//maquina de estados 
	readRobotState(); 
	switch(estado)
	{
		case 0:
		girarHaciaDestino();
		break;

		case 1:
		irDestino();

		case 2:
		esquivarObstaculo();
		break;

		case 4:
		Fin();
		break;

		default:
		break;
	}
}

void SpecificWorker::readRobotState()
{
	try
	{
		differentialrobot_proxy->getBaseState(bState);
		innerModel->updateTransformValues("base", bState.x, 0, bState.z, 0, bState.alpha, 0);
		ldata = laser_proxy->getLaserData();
	}
	catch(const Ice::Exception &e)
	{
		std::cout << "Error reading from Laser" << e << std::endl;
	}
}

void SpecificWorker::girarHaciaDestino()
{
	//transoforma coordenadas al eje de referencia del robot
	QVec vec = innerModel->transform("base",QVec::vec3(destino.x,0,destino.z),"world");	
	float angle = atan2(vec.x(),vec.z());
	this->esquivarObs = 3;

	if(fabs(angle) < 0.05)
	{
		qDebug()<<"salgo de girar hacia destino";
		estado = 1;
		differentialrobot_proxy->setSpeedBase(0,0);
		return;
	}
	
	try{
		qDebug()<<"Giro hacia destino"<<angle;
		differentialrobot_proxy->setSpeedBase(0,angle);
	}
		catch(const Ice::Exception &e)
	{
		std::cout << "Error reading Girar hacia target" << e << std::endl;
	}

}

//sdt::min(iteradores(itBegin[landa], itFin[landa],"Template")   -->mirar cplusplus 
void SpecificWorker::crearrObjetivo()
{
	float v1 = destino.x-bState.x;
	float v2 = destino.z-bState.z;
	rObjetivo.a = v2;
	rObjetivo.b =-v1;
	rObjetivo.c = v1 * bState.z - v2 * bState.x;
	qDebug()<<"          ";
	qDebug()<<"Creo objetivo";
	this->esquivarObs = 3;
}
 bool SpecificWorker::pertenecePunto(float X, float Z)
 {

	 if (fabs((rObjetivo.a*X + rObjetivo.b *Z + rObjetivo.c)) < 100 ){
		 return true;
		 qDebug()<<" estamos en la recta";
	 }
	 return false;
 }

void SpecificWorker::irDestino()
{
	QVec vec = innerModel->transform("base",QVec::vec3(destino.x,0,destino.z),"world");	
	float angle = atan2(vec.x(),vec.z());
	std::sort( ldata.begin(), ldata.end(), [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; });
	if(ldata.front().dist < threshold)
	{
		this->estado=2;
		this->esquivarObs = 1;
	}
	else
	{
		differentialrobot_proxy->setSpeedBase(700, angle);
	}
	if(fabs(bState.x - destino.x) < 100 && fabs(bState.z - destino.z) < 100 )
	{
			this->estado=4;
	}

}

void SpecificWorker::RCISMousePicker_setPick(Pick myPick)
{

	this->estado=0;
	this->destino.x=myPick.x;
	this->destino.z=myPick.z;
	// Guardar recta objetivo
	crearrObjetivo();

}
void SpecificWorker::Fin()
{
	differentialrobot_proxy->setSpeedBase(0,0);
	qDebug()<<"Fin";
	usleep(1000);
}
void SpecificWorker::esquivarObstaculo()
{
	switch (esquivarObs)
	{
	case 1:
		girarObstaculo();
		break;

	case 2:
		rodearObstaculo();
		break;

	default:
		if(this->estado!=4)
			this->estado=0;
		break;
	}
}
void SpecificWorker::girarObstaculo()
{

	differentialrobot_proxy->setSpeedBase(5,rot);
	//calcular distancia de bState al punto objetivo
	coordenada origen;
	origen.x = bState.x;
	origen.z = bState.z;
	this->distanciaEsquivar = distancia(origen,destino); //distancia original entre el robot y el punto objetivo (se usará luego)

	qDebug()<<"Primer giro";
	auto MminI = std::min(ldata.begin()+66, ldata.end()-1, [](auto a, auto b){ return (*a).dist < (*b).dist; }); //sector izquierdo del laser
	float MfloatI = (*MminI).dist;
	qDebug()<<MfloatI;
	if(MfloatI > threshold*2)
	{
		this->esquivarObs = 2;
		differentialrobot_proxy->setSpeedBase(0,0);
	}
}
void SpecificWorker::rodearObstaculo()
{
	//verLinea()
	coordenada origen;
	origen.x = bState.x;
	origen.z = bState.z;
	float dActual = distancia(origen,destino); // dActual = distancia entre robot y e punto actualmente

	if(rObjetivo.pertenece(bState.x,bState.z) && (dActual < distanciaEsquivar  - 100)) // || verDestino()
	{
		//hemos rodeado el objeto, cambiamos de estado
		qDebug()<<"Cambiamos estado";
		differentialrobot_proxy->setSpeedBase(0,0);
		this->estado = 0;
	}
	else
	{
		//comanzamos a esquivar el obstaculo
		//partimos el laser en 3 partes 
		auto minD = std::min(ldata.begin(), ldata.end()-66, [](auto a, auto b){ return (*a).dist < (*b).dist; });
		auto minA = std::min(ldata.begin()+33, ldata.end()-33, [](auto a, auto b){ return (*a).dist < (*b).dist; });
		auto minI = std::min(ldata.begin()+66, ldata.end()-1, [](auto a, auto b){ return (*a).dist < (*b).dist; });
		float floatI = (*minI).dist;
		float floatA = (*minA).dist;
		float floatD = (*minD).dist;
		// qDebug()<<"izquierda"<<floatI;
		// qDebug()<<"adelante"<<floatA;
		// qDebug()<<"derecha"<<floatD;
		if(floatI > threshold*2.2) 
		{
			//girarIzquierda
			qDebug()<<"giro Izquierda";
			differentialrobot_proxy->setSpeedBase(5,-rot);
		}
		else if(floatA > threshold*1.5)
		{
			//ir hacia delante 
			qDebug()<<"Avanzo";
			differentialrobot_proxy->setSpeedBase(700,0);
				usleep(1000);
		}
		else if(floatD > threshold*1.5)
		{
			//girarDerecha
			qDebug()<<"giro Derecha";
			differentialrobot_proxy->setSpeedBase(5,rot);
		}
		else
		{
			//marcha atras
			this->esquivarObs=1;
			qDebug()<<"Marcha atrás";
			differentialrobot_proxy->setSpeedBase(-700,0);

		}
	}
	
	//generar poligono por si vemos destino
	//si lo vemos ir hacia alla y cambiamos estado
	//sino, si hemos llegado a la recta cambiamos estado
		
}
float SpecificWorker::distancia(coordenada origen, coordenada destino)
{
	return sqrt(pow((destino.x - origen.x),2) + pow((destino.z - origen.z),2));
}
bool SpecificWorker::verDestino()
{
	
	
	
	return false;
}



bool SpecificWorker::GotoPoint_atTarget()
{
	return this->fin;
}

void SpecificWorker::GotoPoint_go(string nodo, float x, float y, float alpha)
{
//implementCODE

}

void SpecificWorker::GotoPoint_stop()
{
//implementCODE

}

void SpecificWorker::GotoPoint_turn(float speed)
{
//implementCODE

}
