//============================================================================
// Name        : moveonroad.cpp
// Author      : Bogdan Oancea
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <fstream>

using namespace std;

struct Point {
	double x;
	double y;
};
typedef struct Point point;
int main() {
	point* road;
	road = new point[100];
	road[0] = point { 1, 1 };
	for (int i = 1; i < 100; i++) {
		double dx = rand() % 10 + 1;
		//cout << dx << endl;
		double dy = rand() % 10 + 1;
		if (i <= 25) {
			road[i].x = road[i - 1].x + dx;
			road[i].y = road[i - 1].y + dy;
		} else if (i > 25 && i <= 75) {
			road[i].x = road[i - 1].x + dx;
			road[i].y = road[i - 1].y - dy;
		} else {
			road[i].x = road[i - 1].x + dx;
			road[i].y = road[i - 1].y + dy;
		}
	}
	point* road2;
	road2 = new point[100];
	road2[0] = point { 1, 35 };
	for (int i = 1; i < 100; i++) {
		double dx = rand() % 6 + 1;
		//cout << dx << endl;
		double dy = rand() % 6 + 1;
		if (i <= 15) {
			road2[i].x = road2[i - 1].x + dx;
			road2[i].y = road2[i - 1].y + dy;
		} else if (i > 15 && i <= 50) {
			road2[i].x = road2[i - 1].x + dx;
			road2[i].y = road2[i - 1].y - dy;
		} else {
			road2[i].x = road2[i - 1].x + dx;
			road2[i].y = road2[i - 1].y + dy;
		}
	}

	ofstream street1, street2;
	try {
		street1.open("street1.csv", ios::out);
		street2.open("street2.csv", ios::out);
		for (int i = 0; i < 100; i++) {
			street1 << road[i].x << "," << road[i].y << endl;
			street2 << road2[i].x << "," << road2[i].y << endl;
		}
		street1.close();
		street2.close();
	} catch (ofstream::failure& e) {

		cerr << "Error opening output files!" << endl;
	}

	cout << "!!!Hello World!!!" << endl; // prints !!!Hello World!!!

	double d = 5;
	int N = 100;
	double x0 = 1, y0 = 1, x1, y1;
	int i_on_road = 0;
	ofstream person;
	int step = 0;
	person.open("person_move.csv", ios::out);
	for (int j = 0; j < N; j++) {
		double d_remain = d;
		while (d_remain > 0) {
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

	//go back
	for (int j = 0; j < N; j++) {
		double d_remain = d;
		while (d_remain > 0) {
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
	return 0;
}
