#include "pch.h"
#include "rb-cobot.h"

using namespace rb;


Point::Point(const float x, const float y, const float z, const float rx, const float ry, const float rz) {
	x_ = x;
	y_ = y;
	z_ = z;
	rx_ = rx;
	ry_ = ry;
	rz_ = rz;
}

//Point Point::operator + (Point& p) {
//	x_ = x_ + p.x_;
//	y_ = y_ + p.y_;
//	z_ = z_ + p.z_;
//	rx_ = rx_ + p.rx_;
//	ry_ = ry_ + p.ry_;
//	rz_ = rz_ + p.rz_;
//	return Point(x_, y_, z_, rx_, ry_, rz_);
//}


void Point::operator () (Point p) {
	x_ = p.x_;
	y_ = p.y_;
	z_ = p.z_;
	rx_ = p.rx_;
	ry_ = p.ry_;
	rz_ = p.rz_;
}

void Point::operator () (const float x, const float y, const float z, const float rx, const float ry, const float rz) {
	x_ = x;
	y_ = y;
	z_ = z;
	rx_ = rx;
	ry_ = ry;
	rz_ = rz;
}


Point::~Point() {

}


