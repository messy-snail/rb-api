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
		
	private:
		float x_ = 0.f;
		float y_ = 0.f;
		float z_ = 0.f;
		float rx_ = 0.f;
		float ry_ = 0.f;
		float rz_ = 0.f;

	public:
		Point() {}
		Point(const float x, const float y, const float z, const float rx, const float ry, const float rz);
		Point operator + (Point& p);
		void operator () (const Point p);
		void operator () (const float x, const float y, const float z, const float rx, const float ry, const float rz);
		friend ostream& operator << (ostream& os, rb::Point p) {
			os << "pnt:" << '[' << p.X() << ", " << p.Y() << ", " << p.Z() << ", " << p.RX() << ", " << p.RY() << ", " << p.RZ() << ']';
			return os;
		}
		
		
		inline float X() { return x_; }
		inline float Y() { return y_; }
		inline float Z() { return z_; }
		inline float RX() { return rx_; }
		inline float RY() { return ry_; }
		inline float RZ() { return rz_; }


		virtual ~Point() ;


	};

	


}

#endif