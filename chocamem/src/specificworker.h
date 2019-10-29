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

#include <iostream>
#include <fstream>
#include <genericworker.h>
#include <innermodel/innermodel.h>
#include "grid.h"
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QGraphicsEllipseItem>

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);
	void RCISMousePicker_setPick(const Pick &myPick);


	/// Grid cell definition
		struct TCell
		{
			bool free;
			bool visited;
			QGraphicsRectItem* rect;
			
			// method to save the value
			void save(std::ostream &os) const {	os << free << " " << visited; };
			void read(std::istream &is) {	is >> free >> visited ;};
		};

public slots:
	void compute();
	void initialize(int period);

private:
	std::shared_ptr<InnerModel> innerModel;
	using TDim = Grid<TCell>::Dimensions;
	Grid<TCell> grid;
	QGraphicsScene scene;
	QGraphicsView view;
	void draw();
	QGraphicsRectItem *robot;
	QGraphicsEllipseItem *noserobot;

	int cont;
	int estado;
	int contadorBucle;
	int contadorVisitadas;
	int tilesize = 70;
	int xmin, xmax, ymin, ymax;
	float rot;
	bool giroRecto;

	RoboCompGenericBase::TBaseState bState;
	RoboCompLaser::TLaserData ldata;

	void updateOccupiedCells(const RoboCompGenericBase::TBaseState &bState, const RoboCompLaser::TLaserData &ldata);
	void readRobotState( float threshold);
	void giroIzquierda( float threshold);
	void giroDerecha( float threshold);
	void updateVisitedCells(int x, int z);
	void giroNormal();
	bool checkVisitedCells(int x, int z);
	void giroAntiguo();
	void haciaDelante();
	void giroRandom();
	void barrido();


};

#endif
