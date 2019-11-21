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
const float pi=3.14159265358979323846;


/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	
	RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
	innerModel = std::make_shared<InnerModel>(par.value);
	xmin = std::stoi(params.at("xmin").value);
	xmax = std::stoi(params.at("xmax").value);
	ymin = std::stoi(params.at("ymin").value);
	ymax = std::stoi(params.at("ymax").value);
	tilesize = std::stoi(params.at("tilesize").value);

	// Scene
 	scene.setSceneRect(xmin, ymin, fabs(xmin)+fabs(xmax), fabs(ymin)+fabs(ymax));
 	view.setScene(&scene);
 	view.scale(1, -1);
 	view.setParent(scrollArea);
 	view.fitInView(scene.sceneRect(), Qt::KeepAspectRatio );
	grid.initialize( TDim{ tilesize, xmin, xmax, ymin, ymax}, TCell{true, false, nullptr} );

	qDebug() << "Grid initialize ok";

	for(auto &[key, value] : grid)
 	{
	 	value.rect = scene.addRect(-tilesize/2,-tilesize/2, 100,100, QPen(Qt::NoPen));			
		value.rect->setPos(key.x,key.z);
	}

 	robot = scene.addRect(QRectF(-200, -200, 400, 400), QPen(), QBrush(Qt::blue));
 	noserobot = new QGraphicsEllipseItem(-50,100, 100,100, robot);
 	noserobot->setBrush(Qt::magenta);
	this->estado = 0;
	this->contadorVisitadas=0;
	this->contadorBucle=0;
	this->cont=0;
	this->rot=-pi/4.0;  // rads per second
	this->giroRecto=true;
	view.show();
	showMaximized();
	return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;

	this->Period = period;
	timer.start(Period);
	qDebug() << "End initialize";
	RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();
}

void SpecificWorker::compute()
{ 
// girar 360 grados para conocer obstaculos

	//ldata = laser_proxy->getLaserData();
	bool girar=false;
	const float threshold = 200; // millimeters
	readRobotState(threshold,girar); //si true gira
	/// AQUI LA MAQUINA DE ESTADOS
	switch(estado)
	{
		case 0:
			esperarPick();
		break;

		case 1:
			giroAntiguo();
		break;

		case 2:
			giroDerecha(threshold);
		break;

		case 3:
			haciaDelante();
		break;

		case 4:
			barrido();
		break;

		case 5:
			haciaObjeto();
		break;

		default:
		break;
	}
	
}
	

void SpecificWorker::readRobotState(float threshold, bool girar)
{
	try
	{
		differentialrobot_proxy->getBaseState(bState);
		innerModel->updateTransformValues("base", bState.x, 0, bState.z, 0, bState.alpha, 0);
		ldata = laser_proxy->getLaserData();
		std::sort( ldata.begin(), ldata.end(), [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; });

		if(checkVisitedCells(bState.x, bState.z))  //otra opcion es && this->estado == ?
		{
			contadorVisitadas++;
			qDebug()<<contadorVisitadas;
		}

		//draw robot
		robot->setPos(bState.x, bState.z);
		robot->setRotation(-180*bState.alpha/M_PI);// rot antes era -180

	

	}
	catch(const Ice::Exception &e)
	{
		std::cout << "Error reading from Laser" << e << std::endl;
	}

	//Resize world widget if necessary, and render the world
	if (view.size() != scrollArea->size())
	 		view.setFixedSize(scrollArea->width(), scrollArea->height());

	if(ldata.front().dist < threshold)
	{
		girar = true;
	}
	
	//seleccionar estado
	if(girar)
	{
		this->estado = 0;
		this->contadorBucle++;
		if(contadorBucle>=6)
		{
			contadorBucle=0;
			if(giroRecto)
			{
				rot=2*-pi/6;
				giroRecto=false;
			}
			else
			{
				rot =-pi/4;
				giroRecto=true;
			}
			
		}
	}
	else
	{
		this->estado=3;
	}

	//update  occupied cells
	updateOccupiedCells(bState, ldata);
	updateVisitedCells(bState.x, bState.z);

}
	


void SpecificWorker::updateOccupiedCells(const RoboCompGenericBase::TBaseState &bState, const RoboCompLaser::TLaserData &ldata)
{
	InnerModelLaser *n = innerModel->getNode<InnerModelLaser>(QString("laser"));
	for(auto l: ldata)
	{
		auto r = n->laserTo(QString("world"), l.dist, l.angle);	// r is in world reference system
		// we set the cell corresponding to r as occupied 
		auto [valid, cell] = grid.getCell(r.x(), r.z()); 
		if(valid)
		{
			cell.free = false;
			cell.rect->setBrush(Qt::darkRed);
		}
	}
}


///////////////////////////////////////////////////////////////////77
////  SUBSCRIPTION
/////////////////////////////////////////////////////////////////////

void SpecificWorker::RCISMousePicker_setPick(const Pick &myPick)
{
//subscribesToCODE


}
void SpecificWorker::giroIzquierda(float threshold)
{
    giroNormal();
    giroNormal();

}
void SpecificWorker::haciaDelante()
{
	//avanzar	
    differentialrobot_proxy->setSpeedBase(700, 0);
	//qDebug()<<bState.x<<bState.z;
	
}
void SpecificWorker::giroDerecha(float threshold)
{
	rot =-rot;
    giroNormal();
	rot = -rot;
}
void SpecificWorker::updateVisitedCells(int x, int z)
{
	auto [valid, cell] = grid.getCell(x, z); 
	if(valid)
	{
		if(!cell.visited)
		{
			cont++;
		}
		cell.visited=true;
		//grid.setCell(cell); usar insert (key, value)     hay una funcion para transformar key en x z
		cell.rect->setBrush(Qt::darkBlue);
		float percentOccupacy = 100. * cont / grid.size();
		//auto current = percentOccupacy;
        //lcdNumber->display(percentOccupacy);
		qDebug ()<<"Porcentaje" << percentOccupacy;
	}
}
bool SpecificWorker::checkVisitedCells(int x, int z)
{
	auto [valid, cell] = grid.getCell(x, z); 
	return cell.visited;
}
void SpecificWorker::giroNormal()
{
    differentialrobot_proxy->setSpeedBase(5,rot);
    usleep(1000000);  // wait 1s para esta entreg
}
void SpecificWorker::giroRandom()
{
    differentialrobot_proxy->setSpeedBase(1,rot);
    usleep(rand()%(1500000-100000 + 1) + 100000);  // random wait between 1.5s and 0.1sec
}
void SpecificWorker::giroAntiguo()
{
	differentialrobot_proxy->getBaseState(bState);
    //sort laser data from small to large distances using a lambda function.
    float rot = -pi/4;  // rads per second
    try
    {
        differentialrobot_proxy->setSpeedBase(1, rot);
        giroRandom();
		differentialrobot_proxy->getBaseState(bState);
		updateVisitedCells(bState.x, bState.z);
    }
    catch(const Ice::Exception &ex)
    {
        std::cout << "ERROR EN EL TRY DEL COMPUTE" << std::endl;
    }
	
}
void SpecificWorker::barrido()
{



}

void SpecificWorker:: haciaObjeto()
{

}
