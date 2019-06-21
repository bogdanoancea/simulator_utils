//============================================================================
// Name        : moveonroad.cpp
// Author      : Bogdan Oancea
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <crtdefs.h>
#include <geos/algorithm/LineIntersector.h>
#include <geos/geom/Coordinate.h>
#include <geos/geom/CoordinateArraySequence.h>
#include <geos/geom/GeometryFactory.h>
#include <geos/geom/LineSegment.h>
#include <geos/geom/LineString.h>
#include <geos/geom/MultiPoint.h>
#include <geos/geom/PrecisionModel.h>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>

#define N 100
#define STEPS 200

using namespace std;
using namespace geos;
using namespace geos::geom;
using namespace geos::algorithm;
using namespace geos::io;

struct Point2 {
	double x;
	double y;
};
typedef struct Point2 point;

bool end_of_the_road(int index) {
	if (index == N - 1)
		return true;
	else
		return false;
}

bool start_of_the_road(int index) {
	if (index == 0)
		return true;
	else
		return false;
}

PrecisionModel* pm = new PrecisionModel(2.0, 0.0, 0.0);
GeometryFactory::Ptr m_globalFactory = GeometryFactory::create(pm, -1);

LineString* create_road1(int n) {
	LineString* road;
	CoordinateSequence* cl = new CoordinateArraySequence();
	Coordinate c0(1, 1);
	cl->add(c0);
	for (int i = 1; i < n; i++) {
		double dx = rand() % 10 + 1;
		double dy = rand() % 10 + 1;
		if (i <= 25) {
			double x = cl->getAt(i - 1).x;
			double y = cl->getAt(i - 1).y;
			Coordinate c(x + dx, y + dy);
			cl->add(c);
		} else if (i > 25 && i <= 75) {
			double x = cl->getAt(i - 1).x;
			double y = cl->getAt(i - 1).y;
			Coordinate c(x + dx, y - dy);
			cl->add(c);
		} else {
			double x = cl->getAt(i - 1).x;
			double y = cl->getAt(i - 1).y;
			Coordinate c(x + dx, y + dy);
			cl->add(c);
		}
	}
	road = m_globalFactory->createLineString(cl);
	return road;
}

LineString* create_road2(int n) {
	LineString* road;
	CoordinateSequence* cl = new CoordinateArraySequence();
	Coordinate c0(1, 35);
	cl->add(c0);
	for (int i = 1; i < n; i++) {
		double dx = rand() % 6 + 1;
		double dy = rand() % 6 + 1;
		if (i <= 15) {
			double x = cl->getAt(i - 1).x;
			double y = cl->getAt(i - 1).y;
			Coordinate c(x + dx, y + dy);
			cl->add(c);
		} else if (i > 15 && i <= 50) {
			double x = cl->getAt(i - 1).x;
			double y = cl->getAt(i - 1).y;
			Coordinate c(x + dx, y - dy);
			cl->add(c);
		} else {
			double x = cl->getAt(i - 1).x;
			double y = cl->getAt(i - 1).y;
			Coordinate c(x + dx, y + dy);
			cl->add(c);
		}
	}
	road = m_globalFactory->createLineString(cl);
	return road;
}

double crossProduct(Coordinate a, Coordinate b) {
	return a.x * b.y - b.x * a.y;
}

bool isPointOnLine(LineSegment a, Coordinate b) {
	// Move the image, so that a.first is on (0|0)
	LineSegment aTmp(Coordinate(0.0, 0.0), Coordinate(a.p1.x - a.p0.x, a.p1.y - a.p0.y));
	Coordinate bTmp = Coordinate(b.x - a.p0.x, b.y - a.p0.y);
	double r = crossProduct(aTmp.p1, bTmp);
	return fabs(r) < 0.0000000001;
}

LineString* findCrossingPair(LineString* ls1, LineString* ls2) {
	int indexOfCrossing = 0;
	CoordinateSequence* orig_coords = ls1->getCoordinates();
	for (size_t i = 1; ls1->crosses(ls2) && i < (ls1->getNumPoints() - 1); i++) {
		CoordinateSequence* coords = ls1->getCoordinates();
		coords->deleteAt(i);
		ls1 = m_globalFactory->createLineString(coords);
		indexOfCrossing = i;
	}
	CoordinateSequence* crossingPair = new CoordinateArraySequence();
	for (int j = indexOfCrossing - 1; j < indexOfCrossing + 1; j++) {
		crossingPair->add(orig_coords->getAt(j));
	}

	LineString* result = m_globalFactory->createLineString(crossingPair);
	return result;
}

//https://stackoverflow.com/questions/31520536/compute-intersection-from-multiple-linestrings-using-jts

LineString* addIntersectionPoints(LineString* ls1, Geometry* g) {

	CoordinateSequence* cls = ls1->getCoordinates();
	MultiPoint* mp = dynamic_cast<MultiPoint*>(g);
	CoordinateSequence* cmp = mp->getCoordinates();

	for (size_t i = 0; i < cmp->getSize(); i++) {
		Coordinate c = cmp->getAt(i);
		for (size_t j = 0; j < cls->getSize() - 1; j++) {
			Coordinate c1 = cls->getAt(j);
			Coordinate c2 = cls->getAt(j + 1);
			LineSegment ls(c1, c2);
			Coordinate ret;
			double di = ls.distance(c);
			//if(ret.equals(c)) {
			cout << "punctul de intersectie " << c.toString() << " " << di << " segmentul " << c1.toString() << ":" << c2.toString() << "is point on line " << isPointOnLine(ls, c)
					<< endl;
			//}
		}
	}
	return ls1;
}

int main() {

	LineString* road1 = create_road1(N);
	LineString* road2 = create_road2(N);
	//Geometry* g = road1->intersection(road2);
	cout << " se intersecteaza? " << road1->crosses(road2);
	LineString* l = findCrossingPair(road1, road2);
	cout << l->toString();

	LineIntersector* lineIntersector = new LineIntersector();
	lineIntersector->computeIntersection(Coordinate(64, 74), Coordinate(73, 80), Coordinate(63, 77), Coordinate(68, 72));
	Coordinate intersect = lineIntersector->getIntersection(0);
	cout << intersect << endl;
	ofstream street1, street2;
	try {
		street1.open("street1.csv", ios::out);
		street2.open("street2.csv", ios::out);
		for (int i = 0; i < N; i++) {
			street1 << road1->getCoordinateN(i).x << "," << road1->getCoordinateN(i).y << endl;
			street2 << road2->getCoordinateN(i).x << "," << road2->getCoordinateN(i).y << endl;
		}
		street1.close();
		street2.close();
	} catch (ofstream::failure& e) {
		cerr << "Error opening output files!" << endl;
	}

	//cout << g->toString();
	cout << "!!!Hello World!!!" << endl; // prints !!!Hello World!!!
	/*
	 double d = 5;
	 double x0 = 1, y0 = 1, x1, y1;
	 int i_on_road = 0;
	 ofstream person;
	 int step = 0;
	 person.open("person_move.csv", ios::out);
	 for (int j = 0; j < STEPS; j++) {
	 double d_remain = d;
	 while (d_remain > 0 && !end_of_the_road(i_on_road)) {
	 point p = road[++i_on_road];
	 if (sqrt((p.x - x0) * (p.x - x0) + (p.y - y0) * (p.y - y0)) > d) {
	 double costheta = (p.x - x0) / sqrt((p.x - x0) * (p.x - x0) + (p.y - y0) * (p.y - y0));
	 double sintheta = (p.y - y0) / sqrt((p.x - x0) * (p.x - x0) + (p.y - y0) * (p.y - y0));
	 x1 = x0 + d * costheta;
	 y1 = y0 + d * sintheta;
	 d_remain = 0;
	 i_on_road--;
	 } else {
	 x1 = p.x;
	 y1 = p.y;
	 d_remain = d - sqrt((p.x - x0) * (p.x - x0) + (p.y - y0) * (p.y - y0));
	 }
	 x0 = x1;
	 y0 = y1;
	 step++;
	 person << step << "," << x1 << "," << y1 << endl;
	 }
	 }
	 cout << i_on_road;

	 //go back
	 for (int j = 0; j < STEPS; j++) {
	 double d_remain = d;
	 while (d_remain > 0 && !start_of_the_road(i_on_road)) {
	 point p = road[--i_on_road];
	 if (sqrt((p.x - x0) * (p.x - x0) + (p.y - y0) * (p.y - y0)) > d) {
	 double costheta = (-p.x + x0) / sqrt((p.x - x0) * (p.x - x0) + (p.y - y0) * (p.y - y0));
	 double sintheta = (-p.y + y0) / sqrt((p.x - x0) * (p.x - x0) + (p.y - y0) * (p.y - y0));
	 x1 = x0 - d * costheta;
	 y1 = y0 - d * sintheta;
	 d_remain = 0;
	 i_on_road++;
	 } else {
	 x1 = p.x;
	 y1 = p.y;
	 d_remain = d - sqrt((p.x - x0) * (p.x - x0) + (p.y - y0) * (p.y - y0));
	 }
	 x0 = x1;
	 y0 = y1;
	 step++;
	 person << step << "," << x1 << "," << y1 << endl;
	 }
	 }

	 //now the chalenge is to switch the roads!
	 person.close();
	 */
	return 0;
}
