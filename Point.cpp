#include "pch.h"
#include "Point.h"

using namespace rb;

//Point::Point() {
//
//}

Point::Point(float x, float y, float z, float rx, float ry, float rz) {
	x_ = x;
	y_ = y;
	z_ = z;
	rx_ = rx;
	ry_ = ry;
	rz_ = rz;
	cout << "x: " << x << endl;
	cout << "rz: " << rz << endl;
}

Point::~Point() {

}

