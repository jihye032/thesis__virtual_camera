

#pragma once

#include <vector>
#include "ml.h"

namespace ml
{

class MotionDTW
{
public:
	MotionDTW();
	MotionDTW(const Motion *target, const Motion *input);
	void SetMotions(const Motion *target, const Motion *input);
	//void setMotions2(const PmLinearMotion *base, const PmLinearMotion *input);
	
	double GetDist() { return dist; }

	// int getCorrespAtoB(int i);
	// int getCorrespBtoA(int i);

	// void getWarpedA(PmLinearMotion &outMotion);
	// void getWarpedB(PmLinearMotion &outMotion);
	
	const Motion *target_motion() const { return motion_A_; }
	const Motion *input_motion() const { return motion_B_; }

protected:
	const Motion *motion_A_;
	const Motion *motion_B_;

	double dist;
};

};