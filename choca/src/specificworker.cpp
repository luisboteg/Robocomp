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
SpecificWorker::SpecificWorker(TuplePrx tprx) : GenericWorker(tprx)
{
    cout<<"hola"<<endl;
	rInfo("Specific        ");

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
    	qDebug()<<"hola";

try
	{
        //RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
        //innerModel = std::make_shared<InnerModel>(par.value);
		int xmin = std::stoi(params.at("xmin").value);
		int xmax = std::stoi(params.at("xmax").value);
		int ymin = std::stoi(params.at("ymin").value);
		int ymax = std::stoi(params.at("ymax").value);
		tilesize = std::stoi(params.at("tilesize").value);

		qDebug() << __FILE__ ;
	
		// Scene
		//scene.setSceneRect(-12000, -6000, 38000, 16000);
		scene.setSceneRect(xmin, ymin, fabs(xmin)+fabs(xmax), fabs(ymin)+fabs(ymax));
		view.setScene(&scene);
		view.scale(1, -1);
		//view.setParent(scrollArea);
		//view.setViewport(new QGLWidget(QGLFormat(QGL::SampleBuffers)));
		view.fitInView(scene.sceneRect(), Qt::KeepAspectRatio );

		//grid.initialize( TDim{ tilesize, -12000, 25000, -6000, 10000}, TCell{0, true, false, nullptr, 0.} );
		grid.initialize( TDim{ tilesize, xmin, xmax, ymin, ymax}, TCell{0, true, false, nullptr, 0.} );
		
		for(auto &[key, value] : grid)
		{
			auto tile = scene.addRect(-tilesize/2,-tilesize/2, 100,100, QPen(Qt::NoPen));
			tile->setPos(key.x,key.z);
			value.rect = tile;
		}

		robot = scene.addRect(QRectF(-200, -200, 400, 400), QPen(), QBrush(Qt::blue));
		noserobot = new QGraphicsEllipseItem(-50,100, 100,100, robot);
		noserobot->setBrush(Qt::magenta);

		target = QVec::vec3(0,0,0);
		
		//qDebug() << __FILE__ << __FUNCTION__ << "CPP " << __cplusplus;
		
		
		view.show();

		//defaultMachine.start();
		//QStateMachine.start();
		
		return true;
	}

	catch(const std::exception &e) { qFatal("Error reading config params"); }

}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;
	this->Period = period;
	timer.start(Period);

}

void SpecificWorker::compute()
{

    RoboCompGenericBase::TBaseState bState;
    const float threshold = 200; // millimeters
    float rot = -pi/4;  // rads per second
    try
    {
        // read laser data
        RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();

        differentialrobot_proxy->getBaseState(bState);
        //innerModel->updateTransformValues("base", bState.x, 0, bState.z, 0, bState.alpha, 0);

        //sort laser data from small to large distances using a lambda function.
        std::sort( ldata.begin(), ldata.end(), [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; });

		updateVisitedCells(bState.x, bState.z);
		updateOccupiedCells(bState, ldata);

        if( ldata.front().dist < threshold )
        {
                cout<<"debug1, girando"<<endl;
                differentialrobot_proxy->setSpeedBase(5, rot);
                std::cout << ldata.front().dist << std::endl;
                //giroNormal(rot);
                giroRandom(0.1);
        }
        else
        {
                differentialrobot_proxy->setSpeedBase(700, 0);
                //anadirLista(rot);
        }
    }
    catch(const Ice::Exception &ex)
    {
        std::cout << "ERROR EN EL TRY DEL COMPUTE" << std::endl;
    }
}
void SpecificWorker::giroRandom(float rot)
{
    usleep(rand()%(1500000-100000 + 1) + 100000);  // random wait between 1.5s and 0.1sec
}
void SpecificWorker::giroNormal(float rot)
{
    cout<<"debug2"<<endl;
    //differentialrobot_proxy->setSpeedBase(5, rot);
    usleep(1000000);  // wait 1s
    // this->apunta++;
    // if(this->apunta == 8)
    // {
    //     this->apunta = 0;
    // }
}

void SpecificWorker::checkTransform(const RoboCompGenericBase::TBaseState &bState)
{
	//auto r = innerModel->transform("base", target, "world");		// using InnerModel
	
	Rot2D rot(bState.alpha);																		// create a 2D clockwise rotation matrix
	QVec t = QVec::vec2(bState.x, bState.z);									  // create a 2D vector for robot translation
	QVec t2 = QVec::vec2(target.x(), target.z());								// create a 2D vector from the 3D target
	QVec q = rot.transpose() * ( t2 - t);												// multiply R_t * (y - T)
	//qDebug() << target << r << q;
}

void SpecificWorker::updateOccupiedCells(const RoboCompGenericBase::TBaseState &bState, const RoboCompLaser::TLaserData &ldata)
{
	//InnerModelLaser *n = innerModel->getNode<InnerModelLaser>(QString("laser"));
	for(auto l: ldata)
	{
	//	auto r = n->laserTo(QString("world"), l.dist, l.angle);	// r is in world reference system
		// we set the cell corresponding to r as occupied 
	//	auto [valid, cell] = grid.getCell(r.x(), r.z()); 
		// if(valid)
		// {
		// 	cell.free = false;
		// 	cell.rect->setBrush(Qt::darkRed);
		// }
	}
}

void SpecificWorker::updateVisitedCells(int x, int z)
{
	static unsigned int cont = 0;
	auto [valid, cell] = grid.getCell(x, z); 
	if(valid)
	{
		auto &occupied = cell.visited;
		if(occupied)
		{
			occupied = false;
			cont++;
		}
		float percentOccupacy = 100. * cont / grid.size();
        cout<<percentOccupacy<<endl;
	}
}
// void SpecificWorker::draw()
// {
// 	for(auto &[key, value] : grid)
// 	{
// // 		if(value.visited == false)
// // 			value.rect->setBrush(Qt::lightGray);
// 		if(value.free == false)
// 			value.rect->setBrush(Qt::darkRed);
// 	}
// 	view.show();
// }