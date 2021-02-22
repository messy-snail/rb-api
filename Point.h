#ifndef POINT_H
#define POINT_H

#ifdef RB_EXPORTS
#define POINT_DECLSPEC __declspec(dllexport)
#else
#define POINT_DECLSPEC __declspec(dllimport)
#endif

#include "CommonHeader.h"
using namespace std;

namespace rb {
	class POINT_DECLSPEC Point
	{
	public:
		//Point();
		Point(float x, float y, float z, float rx, float ry, float rz);
		Point operator + (Point& p) {
			x_ = x_ + p.x_;
			y_ = y_ + p.y_;
			z_ = z_ + p.z_;
			rx_ = rx_ + p.rx_;
			ry_ = ry_ + p.ry_;
			rz_ = rz_ + p.rz_;
			return Point(x_, y_, z_, rx_, ry_, rz_);
		}
		friend ostream& operator<<(ostream&, const Point&);

		inline float X() { return x_; }
		inline float Y() { return y_; }
		inline float Z() { return z_; }
		inline float RX() { return rx_; }
		inline float RY() { return ry_; }
		inline float RZ() { return rz_; }


		virtual ~Point() ;

	private:
		float x_ = 0.f;
		float y_ = 0.f;
		float z_ = 0.f;
		float rx_ = 0.f;
		float ry_ = 0.f;
		float rz_ = 0.f;
	};

	ostream& operator<<(ostream& os, const Point& p)
	{
		os << "pnt:"<<'[' << p.x_ << ", " << p.y_ << ", " << p.z_ << ", " << p.rx_ << ", " << p.ry_ << ", " << p.rz_ << ']' << endl;
		return os;
	}
}

#endif