#include "pch.h"
#include "rb-cobot.h"

using namespace rb;

Joint::Joint(const float j0, const float j1, const float j2, const float j3, const float j4, const float j5) {
	j0_= j0;
	j1_= j1;
	j2_= j2;
	j3_ = j3;
	j4_ = j4;
	j5_ = j5;
}

Joint Joint::operator + (const Joint& j) {
	float j_result0	= j0_ + j.j0_;
	float j_result1	= j1_ + j.j1_;
	float j_result2	= j2_ + j.j2_;
	float j_result3 = j3_ + j.j3_;
	float j_result4 = j4_ + j.j4_;
	float j_result5 = j5_ + j.j5_;
	return Joint(j_result0, j_result1, j_result2, j_result3, j_result4, j_result5);
}

Joint Joint::operator - (const Joint& j) {
	float j_result0 = j0_ - j.j0_;
	float j_result1 = j1_ - j.j1_;
	float j_result2 = j2_ - j.j2_;
	float j_result3 = j3_ - j.j3_;
	float j_result4 = j4_ - j.j4_;
	float j_result5 = j5_ - j.j5_;
	return Joint(j_result0, j_result1, j_result2, j_result3, j_result4, j_result5);
}

Joint::~Joint() {

}


