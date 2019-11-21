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
		innerModel = std::make_shared<InnerModel>( "/home/robocomp/robocomp/files/innermodel/simpleworld.xml" );
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
	this->estado=4;
	this->threshold=200;
	this->rot = 0.6;
}

void SpecificWorker::compute()
{ 
	readRobotState(); 
	/// AQUI LA MAQUINA DE ESTADOS
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

		case 3:
		volverCamino();
		break;

		case 4:
		volverCamino();
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
	//transofrma coordenadas al eje de referencia del robot
	QVec vec = innerModel->transform("base",QVec::vec3(destino.x,0,destino.z),"world");
	float angle = atan2(vec.x(),vec.z());

	if(fabs(angle) < 0.05)
	{
		estado = 1;
		differentialrobot_proxy->setSpeedBase(0,0);

		return;
	}
	try{
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
	 if ((rObjetivo.a*X + rObjetivo.b *Z + rObjetivo.c)){
		 return true;
	 }
	 return false;
 }

void SpecificWorker::irDestino() //añadir comprobacion de error por el angulo alpha
{
	std::sort( ldata.begin(), ldata.end(), [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; });
	qDebug()<<"            ";
	qDebug()<<"Voy destino"<<ldata.front().dist;
	qDebug()<<"            ";
	if(ldata.front().dist < threshold)
	{
		this->estado=2;
		this->esquivarObs = 1;
	}
	else
	{
		differentialrobot_proxy->setSpeedBase(700, 0);
	}

	if(fabs(bState.x - destino.x) < 100 && fabs(bState.z - destino.z) < 100 )
	{
		differentialrobot_proxy->setSpeedBase(0,0);
		this->estado=5;
	}

}
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

void SpecificWorker::RCISMousePicker_setPick(Pick myPick)
{
//subscribesToCODE
	this->estado=0;
	this->destino.x=myPick.x;
	this->destino.z=myPick.z;
	// Guardar recta objetivo
	crearrObjetivo();

}
void SpecificWorker::volverCamino()
{


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
		this->estado=0;
		break;
	}


//sino girar segun lo leido
}
void SpecificWorker::girarObstaculo()
{
	// auto izquierda = ldata[19];
	// auto derecha = ldata[81];//acumuladores de cada lado
	// //mirar laser para girar
	// for(int i= 20; i < ldata.size()-20 ; i++) //ldata.size() = 100
	// {
	// 	if(i < ldata.size()/2)
	// 	{
	// 		izquierda += ldata[i];
	// 	}
	// 	else
	// 	{
	// 		derecha += ldata[i];
	// 	}
		
	// }
	// EN PRINCIPIO VAMOS A GIRAR DERECHA

	//si estamos paralelo al obstaculo cambiamos de estado
	//en principio giramos 90 grados

	differentialrobot_proxy->setSpeedBase(5,rot);
	//usleep(1000000);
	//calcular distancia de bState al punto destino
	coordenada origen;
	origen.x = bState.x;
	origen.z = bState.z;
	this->distanciaEsquivar = distancia(origen,destino);

	qDebug()<<"Primer giro";
	auto MminI = std::min(ldata.begin()+66, ldata.end()-1, [](auto a, auto b){ return (*a).dist < (*b).dist; });
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
	//mirar laser
	auto minD = std::min(ldata.begin(), ldata.end()-50, [](auto a, auto b){ return (*a).dist < (*b).dist; });
	auto minA = std::min(ldata.begin()+30, ldata.end()-30, [](auto a, auto b){ return (*a).dist < (*b).dist; });
	auto minI = std::min(ldata.begin()+50, ldata.end()-1, [](auto a, auto b){ return (*a).dist < (*b).dist; });
	float floatI = (*minI).dist;
	float floatA = (*minA).dist;
	float floatD = (*minD).dist;
	qDebug()<<"izquierda"<<floatI;
	qDebug()<<"adelante"<<floatA;
	qDebug()<<"derecha"<<floatD;
	if(floatI > threshold*2) 
	{
		//girarIzquierda
		qDebug()<<"giro Izquierda";
		differentialrobot_proxy->setSpeedBase(5,-rot);
		//usleep(1000000);
		//differentialrobot_proxy->setSpeedBase(700,0);
	}
	else if(floatA > threshold*2)
	{
		//ir hacia delante 
		qDebug()<<"Avanzo";
		differentialrobot_proxy->setSpeedBase(700,0);
	}
	else if(floatD > threshold*2)
	{
		//girarDerecha
		qDebug()<<"giro Derecha";
		differentialrobot_proxy->setSpeedBase(5,rot);
		//usleep(1000000);
		//differentialrobot_proxy->setSpeedBase(700,0);
	}
	else
	{
		//dar la vuelta
		this->esquivarObs=1;
		//usleep(2*1000000);
		qDebug()<<"Vuelvo primer giro";
		//differentialrobot_proxy->setSpeedBase(700,0);

	}
	
	//MIRAR NOS CARGAMOS EL VECTOR DEL LASER (IDEA)
	//HACER QDEBUG EN ESTADO IR DESTINO CUANDO GIRA



	//verObjetivo()
	//girarObjetivo
	//verLinea()

	
	//generar poligono por si vemos destino
	//si lo vemos ir hacia alla y cambiamos estado
	//sino, si hemos llegado a la recta cambiamos estado
		
	//comprobar laser izquierda para ver si pasamos el  objeto (revisar distancia)
	//si lo hemos pasado girar izquierda (por ahora seran 90 grados)
}
float SpecificWorker::distancia(coordenada origen, coordenada destino)
{
	return sqrt(pow((destino.x - origen.x),2) + pow((destino.z - origen.z),2));
}