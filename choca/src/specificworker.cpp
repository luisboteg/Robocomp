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
//	To use innerModelPath parameter you should uncomment specificmonitor.cpp readConfig method content
//	try
//	{
//		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
//		std::string innermodel_path = par.value;
//		innerModel = new InnerModel(innermodel_path);
//	}
//	catch(std::exception e) { qFatal("Error reading config params"); }



	


	return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;
	this->Period = period;
	timer.start(Period);
//     mycoordenada start;
//     start.x=0;
//     start.y=0;
//  //   start.angulo=pi/2;
//     start.direccion=2;
    //lista.push_back(start);
}

void SpecificWorker::compute()
{

//    RoboCompGenericBase::TBaseState bState;
    const float threshold = 200; // millimeters
    float rot = -pi/4;  // rads per second
    try
    {
        // read laser data
        RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();
        // differentialrobot_proxy->getBaseState(bState);
        // auto current = fm.addStep(bState.x, bState.z, bState.alpha);
        // lcdNumber->display(current);
        //sort laser data from small to large distances using a lambda function.
        std::sort( ldata.begin(), ldata.end(), [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; });

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
    this->apunta++;
    if(this->apunta == 8)
    {
        this->apunta = 0;
    }
}
// void SpecificWorker::anadirLista(float rot)
// {
//     cout<<"Entro en añadir lista";
//     mycoordenada nueva;
//     mycoordenada anterior = lista.back();
//     nueva.x = anterior.x;
//     nueva.y = anterior.y;
//  //   nueva.angulo = anterior.angulo;
//     nueva.direccion = this->apunta;

//     switch(nueva.direccion)
//     {
//         case 0:
//             nueva.x++;
//         //    nueva.angulo = 0;
//         break;

//         case 1:
//             nueva.x++;
//             nueva.y++;
//         //    nueva.angulo = pi/4;
//         break;
//         case 2:
//             nueva.y++;
//         //    nueva.angulo = pi/2;
//         break;
//         case 3:
//             nueva.x--;
//             nueva.y++;
//         //    nueva.angulo = 3*pi/4;

//         break;
//         case 4:
//             nueva.x--;
//         //    nueva.angulo = pi;
//         break;
//         case 5:
//             nueva.x--;
//             nueva.y--;
//         //    nueva.angulo = 5*pi/4;
//         break;
//         case 6:
//             nueva.y--;
//         //    nueva.angulo = 3*pi/2;
//         break;
//         case 7:
//             nueva.x++;
//             nueva.y--;
//         //    nueva.angulo = 7*pi/4;
//         break;        
//         case 8:
//             nueva.direccion = 0;
//             nueva.x++;
//         //    nueva.angulo = 0;
//         break;
//         default:
//         break;
//     }
//         lista.push_back(nueva);
//         cout<<"mostrando"<<endl;
//         cout<<nueva.x<<"  "<<nueva.y;

// }
// bool SpecificWorker::estaEnLista()
// {
//     mycoordenada nueva = lista[lista.size()-1];
//     nueva.direccion = this->apunta;

//     switch(nueva.direccion)
//     {
//         case 0:     
//             nueva.x++;
//         break;

//         case 1:
//             nueva.x++;
//             nueva.y++;
//         break;
//         case 2:
//             nueva.y++;
//         break;
//         case 3:
//             nueva.x--;
//             nueva.y++;

//         break;
//         case 4:
//             nueva.x--;
//         break;
//         case 5:
//             nueva.x--;
//             nueva.y--;
//         break;
//         case 6:
//             nueva.y--;
//         break;
//         case 7:
//             nueva.x++;
//             nueva.y--;
//         break;        
//         case 8:
//             nueva.direccion = 0;
//             nueva.x++;
//         break;
//         default:
//         break;
//     }
  //  if(lista.)
//}
// void SpecificWorker::resetSlot()
// {
//     fm.reset();
// }
// USAR GRID.H