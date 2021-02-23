#include "pch.h"
#include "Joint.h"

using namespace rb;

Joint::Joint(const float j0, const float j1, const float j2, const float j3, const float j4, const float j5) {
	j0_= j0;
	j1_= j1;
	j2_= j2;
	j3_ = j3;
	j4_ = j4;
	j5_ = j5;
}

Joint::~Joint() {

}


