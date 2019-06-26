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
#include <utility>
#include <exception>

#define N 100
#define STEPS 200

using namespace std;
using namespace geos;
using namespace geos::geom;
using namespace geos::algorithm;
using namespace geos::io;

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

pair<LineString*, LineString*> findCrossingPair(LineString* ls1, LineString* ls2) {
	CoordinateSequence* cls1 = ls1->getCoordinates();
	CoordinateSequence* cls2 = ls2->getCoordinates();
	CoordinateSequence* new_cls1 = new CoordinateArraySequence();
	CoordinateSequence* new_cls2 = new CoordinateArraySequence();
	vector<Coordinate> intersection_points;
	vector<LineString*> intersection_segments2;

	// ls1 with ls2
	for (size_t i = 0; i < ls1->getNumPoints() - 1; i += 1) {
		Coordinate cls1_c1 = cls1->getAt(i);
		Coordinate cls1_c2 = cls1->getAt(i + 1);
		CoordinateSequence* crossingPair1 = new CoordinateArraySequence();
		crossingPair1->add(cls1_c1);
		crossingPair1->add(cls1_c2);
		if (i == 0)
			new_cls1->add(cls1_c1);
		LineString* segm1 = m_globalFactory->createLineString(crossingPair1);
		if (segm1->crosses(ls2)) {
			for (size_t j = 0; j < ls2->getNumPoints() - 1; j += 1) {
				Coordinate cls2_c1 = cls2->getAt(j);
				Coordinate cls2_c2 = cls2->getAt(j + 1);
				CoordinateSequence* crossingPair2 = new CoordinateArraySequence();
				crossingPair2->add(cls2_c1);
				crossingPair2->add(cls2_c2);
				LineString* segm2 = m_globalFactory->createLineString(crossingPair2);
				if (segm1->crosses(segm2)) {
					LineIntersector* lineIntersector = new LineIntersector();
					lineIntersector->computeIntersection(cls1_c1, cls1_c2, cls2_c1, cls2_c2);
					Coordinate intersect = lineIntersector->getIntersection(0);
					intersection_points.push_back(intersect);
					intersection_segments2.push_back(segm2);
					new_cls1->add(intersect);
				}
			}
		}
		new_cls1->add(cls1_c2);
	}
	for (size_t i = 0; i < ls2->getNumPoints(); i++) {
		Coordinate c = cls2->getAt(i);
		new_cls2->add(c);
		for (size_t k = 0; k < intersection_points.size(); k++) {
			const Coordinate intersect = intersection_segments2[k]->getCoordinateN(0);
			if (c.equals(intersect)) {
				new_cls2->add(intersection_points.at(k));
			}
		}
	}
	// ls2 with ls1
//	for (size_t i = 0; i < ls2->getNumPoints() - 1; i += 1) {
//		Coordinate cls2_c1 = cls2->getAt(i);
//		Coordinate cls2_c2 = cls2->getAt(i + 1);
//		CoordinateSequence* crossingPair2 = new CoordinateArraySequence();
//		crossingPair2->add(cls2_c1);
//		crossingPair2->add(cls2_c2);
//		if (i == 0)
//			new_cls2->add(cls2_c1);
//		LineString* segm2 = m_globalFactory->createLineString(crossingPair2);
//		if (segm2->crosses(ls1)) {
//			for (size_t j = 0; j < ls1->getNumPoints() - 1; j += 1) {
//				Coordinate cls1_c1 = cls1->getAt(j);
//				Coordinate cls1_c2 = cls1->getAt(j + 1);
//				CoordinateSequence* crossingPair1 = new CoordinateArraySequence();
//				crossingPair1->add(cls1_c1);
//				crossingPair1->add(cls1_c2);
//				LineString* segm1 = m_globalFactory->createLineString(crossingPair1);
//				if (segm2->crosses(segm1)) {
//					LineIntersector* lineIntersector = new LineIntersector();
//					lineIntersector->computeIntersection(cls2_c1, cls2_c2, cls1_c1, cls1_c2);
//					Coordinate intersect = lineIntersector->getIntersection(0);
//					cout << " intersectia la: " << intersect.toString() << endl;
//					new_cls2->add(intersect);
//				}
//			}
//		}
//		new_cls2->add(cls2_c2);
//	}
	LineString* newls1 = m_globalFactory->createLineString(new_cls1);
	LineString* newls2 = m_globalFactory->createLineString(new_cls2);

	pair<LineString*, LineString*> result = make_pair(newls1, newls2);
	return result;
}

//https://stackoverflow.com/questions/31520536/compute-intersection-from-multiple-linestrings-using-jts

int findPointOnRoad(LineString* road, Coordinate start) {
	int result = -1;
	CoordinateSequence* coords = road->getCoordinates();
	cout << " road coordinates " << coords->getSize() << endl;
	cout << " road points " << road->getNumPoints() << endl;
	for (size_t j = 0; j < road->getNumPoints(); j++) {
		cout << j << " " << coords->getAt(j) << endl;
		if (coords->getAt(j).equals(start)) {
			//cout << j << " " << coords->getAt(j) << endl;
			result = j;
			break;
		}
	}
	return result;
}

CoordinateSequence* move_person_on_road(double step_length, Coordinate start, LineString* road, int NSTEPS) {
	CoordinateSequence* result = new CoordinateArraySequence();

	double x0 = start.x, y0 = start.y, x1, y1;
	ofstream person;
	person.open("person_move.csv", ios::out);

	int i_on_road = findPointOnRoad(road, start);
	if (i_on_road == -1)
		throw "start point not on road";

	int step_no = 0;
	while (step_no < NSTEPS) {
		// go forward
		double d_remain = step_length;
		while (d_remain > 0 && !end_of_the_road(i_on_road)) {
			Coordinate p = road->getCoordinateN(++i_on_road);
			if (sqrt((p.x - x0) * (p.x - x0) + (p.y - y0) * (p.y - y0)) > step_length) {
				double costheta = (p.x - x0) / sqrt((p.x - x0) * (p.x - x0) + (p.y - y0) * (p.y - y0));
				double sintheta = (p.y - y0) / sqrt((p.x - x0) * (p.x - x0) + (p.y - y0) * (p.y - y0));
				x1 = x0 + step_length * costheta;
				y1 = y0 + step_length * sintheta;
				d_remain = 0;
				i_on_road--;
			} else {
				x1 = p.x;
				y1 = p.y;
				d_remain = step_length - sqrt((p.x - x0) * (p.x - x0) + (p.y - y0) * (p.y - y0));
			}
			x0 = x1;
			y0 = y1;
			step_no++;
			person << step_no << "," << x1 << "," << y1 << endl;
		}
		// go backward
		d_remain = step_length;
		while (d_remain > 0 && !start_of_the_road(i_on_road)) {
			Coordinate p = road->getCoordinateN(--i_on_road);
			if (sqrt((p.x - x0) * (p.x - x0) + (p.y - y0) * (p.y - y0)) > step_length) {
				double costheta = (-p.x + x0) / sqrt((p.x - x0) * (p.x - x0) + (p.y - y0) * (p.y - y0));
				double sintheta = (-p.y + y0) / sqrt((p.x - x0) * (p.x - x0) + (p.y - y0) * (p.y - y0));
				x1 = x0 - step_length * costheta;
				y1 = y0 - step_length * sintheta;
				d_remain = 0;
				i_on_road++;
			} else {
				x1 = p.x;
				y1 = p.y;
				d_remain = step_length - sqrt((p.x - x0) * (p.x - x0) + (p.y - y0) * (p.y - y0));
			}
			x0 = x1;
			y0 = y1;
			step_no++;
			person << step_no << "," << x1 << "," << y1 << endl;
		}
	}
	person.close();
	return result;
}

LineString* isPointOnOtherRoad(Coordinate c, LineString* road, vector<LineString*>& all_roads) {
	int index = -1;
	LineString* result = nullptr;
	for (size_t i = 0; i < all_roads.size(); i++) {
		LineString* r = all_roads.at(i);
		if(!road->equals(r)) {
			cout << " drumul " << r->toString() << endl;
			cout << " punctul " << c.toString() << endl;
			index = findPointOnRoad(r, c);
			if(index != -1) {
				result = r;
				break;
			}
		}
	}
	//cout << " drumul inainte de return " << result->toString() << endl;
	return result;
}


CoordinateSequence* move_person_on_road_switch(double step_length, Coordinate start, LineString* road, vector<LineString*>& all_roads, int NSTEPS) {
	CoordinateSequence* result = new CoordinateArraySequence();

	double x0 = start.x, y0 = start.y, x1, y1;
	ofstream person;
	int i_on_road = findPointOnRoad(road, start);
	if (i_on_road == -1)
		throw "start point not on road";

	person.open("person_move.csv", ios::out);

	int step_no = 0;
	while (step_no < NSTEPS) {
		// go forward
		double d_remain = step_length;
		while (d_remain > 0 && !end_of_the_road(i_on_road)) {
			Coordinate p = road->getCoordinateN(++i_on_road);
			if (sqrt((p.x - x0) * (p.x - x0) + (p.y - y0) * (p.y - y0)) > step_length) {
				double costheta = (p.x - x0) / sqrt((p.x - x0) * (p.x - x0) + (p.y - y0) * (p.y - y0));
				double sintheta = (p.y - y0) / sqrt((p.x - x0) * (p.x - x0) + (p.y - y0) * (p.y - y0));
				x1 = x0 + step_length * costheta;
				y1 = y0 + step_length * sintheta;
				d_remain = 0;
				i_on_road--;
			} else {
				x1 = p.x;
				y1 = p.y;
				d_remain = step_length - sqrt((p.x - x0) * (p.x - x0) + (p.y - y0) * (p.y - y0));
				//check if p is on other road and switch
				LineString* other = isPointOnOtherRoad(p, road, all_roads);
				if(other != nullptr) {
					cout << " in move person " << p.toString() << " : " << other->toString() << endl;
					road = other;
					i_on_road = findPointOnRoad(other, p);
					//update i_on_road
				}
			}
			x0 = x1;
			y0 = y1;
			step_no++;
			person << step_no << "," << x1 << "," << y1 << endl;
		}
		// go backward
		d_remain = step_length;
		while (d_remain > 0 && !start_of_the_road(i_on_road)) {
			Coordinate p = road->getCoordinateN(--i_on_road);
			if (sqrt((p.x - x0) * (p.x - x0) + (p.y - y0) * (p.y - y0)) > step_length) {
				double costheta = (-p.x + x0) / sqrt((p.x - x0) * (p.x - x0) + (p.y - y0) * (p.y - y0));
				double sintheta = (-p.y + y0) / sqrt((p.x - x0) * (p.x - x0) + (p.y - y0) * (p.y - y0));
				x1 = x0 - step_length * costheta;
				y1 = y0 - step_length * sintheta;
				d_remain = 0;
				i_on_road++;
			} else {
				x1 = p.x;
				y1 = p.y;
				d_remain = step_length - sqrt((p.x - x0) * (p.x - x0) + (p.y - y0) * (p.y - y0));
			}
			x0 = x1;
			y0 = y1;
			step_no++;
			person << step_no << "," << x1 << "," << y1 << endl;
		}
	}
	person.close();
	return result;
}

int main() {
	LineString* road1 = create_road1(N);
	LineString* road2 = create_road2(N);
	pair<LineString*, LineString*> l = findCrossingPair(road1, road2);
	ofstream street1, street2;
	try {
		street1.open("street1.csv", ios::out);
		street2.open("street2.csv", ios::out);
		for (size_t i = 0; i < l.first->getNumPoints(); i++) {
			street1 << l.first->getCoordinateN(i).x << "," << l.first->getCoordinateN(i).y << endl;
			street2 << l.second->getCoordinateN(i).x << "," << l.second->getCoordinateN(i).y << endl;
		}
		street1.close();
		street2.close();
	} catch (ofstream::failure& e) {
		cerr << "Error opening output files!" << endl;
	}
	vector<LineString*> all_roads;
	all_roads.push_back(l.first);
	all_roads.push_back(l.second);
	try {
		//move_person_on_road(25, Coordinate(1, 1), road1, 600);
		move_person_on_road_switch(25, Coordinate(1, 1), l.first, all_roads, 600);
	} catch (const char* msg) {
		cout << msg;
	}
	return 0;
}
