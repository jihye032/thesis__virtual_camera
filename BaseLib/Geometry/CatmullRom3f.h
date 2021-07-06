
#ifndef CATMULL_ROM_3F_H
#define CATMULL_ROM_3F_H


#include "BaseLib/CmlExt/CmlExt.h"


#include <vector>

namespace mg
{

class CatmullRom3f
{
public:
	CatmullRom3f();

	void clear();

	void setCtrlPoint(int index, cml::vector3d p);
	void addCtrlPoint(cml::vector3d pos);

	int getCtrlSize();

	cml::vector3d get(int index, float t);

	int getSplineSize(int density);
	void getSpline(cml::vector3d* p, int density);


private:
	std::vector<cml::vector3d> posList;

	int lastRefIndex;

	cml::vector3d b1, b2, b3, b4;
};




};

#endif