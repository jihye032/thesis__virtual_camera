
#include "BaseLib/Geometry/CatmullRom3f.h"
#include "BaseLib/Geometry/bezier.h"

namespace mg
{

CatmullRom3f::CatmullRom3f()
{
	lastRefIndex = -1;
}

void CatmullRom3f::clear()
{
	posList.clear();
}


void CatmullRom3f::setCtrlPoint(int index, cml::vector3d pos)
{
	if ( index <=  (int)posList.size() )
		posList.resize(index+1);


	posList[index] = pos;
}



void CatmullRom3f::addCtrlPoint(cml::vector3d pos)
{
	posList.push_back(pos);
}



int CatmullRom3f::getCtrlSize()
{
	return (int)posList.size();
}


cml::vector3d CatmullRom3f::get(int index, float t)
{
	int size = (int)posList.size();
	
	if ( index >= size || index < 0 ) return cml::vector3d(0, 0, 0);
	else if ( size == 1 ) return posList[0];

	cml::vector3d v1, v2;

	
	if ( lastRefIndex != index )
	{
		b1 = posList[index];
		b4 = posList[index+1];


		if ( index == 0 )
		{
			v1 = (b4 - b1) / 2;
		}
		else
		{
			v1 = (b4 - posList[index-1]) / 2;
		}
		b2 = (v1/3) + b1;
			
		if ( index == size-2 )
		{	
			v2 = (b4 - b1) / 2;
		}
		else 
		{
			v2 = (posList[index+2] - b1) / 2;
		}
		b3 = b4 - (v2/3);

		lastRefIndex = index;
	}

	return BezierCurver3d::Evalutate(b1, b2, b3, b4, (double)t);
}

int CatmullRom3f::getSplineSize(int density)
{
	if ( getCtrlSize() <= 0 ) return 0;
	return (this->getCtrlSize()-1) * density + 1;
}

void CatmullRom3f::getSpline(cml::vector3d* p, int density)
{
	if ( getCtrlSize() <= 0 ) return;

	float gap = 1.0f / density;
	float t;
	
	int size = (int)this->getCtrlSize();

	
	for ( int i = 0; i < size-1; i++ )
	{
		for ( int j = 0; j < density; j++ )
		{
			t = j * gap;
			p[(i*density) + j] = this->get(i, t);
		}
	}
	p[(size-1)*density] = posList[size-1];
	
}





};