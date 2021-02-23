#ifndef JOINT_H
#define JOINT_H

#ifdef RB_EXPORTS
#define JOINT_DECLSPEC __declspec(dllexport)
#else
#define JOINT_DECLSPEC __declspec(dllimport)
#endif

#include "CommonHeader.h"
using namespace std;

namespace rb {
	class JOINT_DECLSPEC Joint
	{
	private:
		float j0_ = 0.f;
		float j1_ = 0.f;
		float j2_ = 0.f;
		float j3_ = 0.f;
		float j4_ = 0.f;
		float j5_ = 0.f;
	public:
		Joint() {}
		Joint(const float j0, const float j1, const float j2, const float j3, const float j4, const float j5);
		Joint operator + (Joint& j);
		Joint operator - (Joint& j);
		friend ostream& operator << (ostream& os, rb::Joint j) {
			os << "jnt:" << '[' << j.J0() << ", " << j.J1() << ", " << j.J2() << ", " << j.J3() << ", " << j.J4() << ", " << j.J5() << ']';
			return os;
		}

		virtual ~Joint();

		inline float J0() { return j0_; }
		inline float J1() { return j1_; }
		inline float J2() { return j2_; }
		inline float J3() { return j3_; }
		inline float J4() { return j4_; }
		inline float J5() { return j5_; }

	};
}


#endif