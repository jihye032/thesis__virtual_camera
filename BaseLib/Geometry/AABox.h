
#pragma once
#include "BaseLib/CmlExt/CmlExt.h"


namespace mg
{

class AABox
{
public:
	
	void minmax_corner(cml::vector3d mi, cml::vector3d ma) { min_corner_ = mi; max_corner_ = ma; }
	void min_corner(cml::vector3d m) { min_corner_ = m; }
	void max_corner(cml::vector3d m) { max_corner_ = m; }

	cml::vector3d min_corner() const    { return min_corner_; }
	cml::vector3d max_corner() const    { return max_corner_; }
	cml::vector3d center() const { return (max_corner_+min_corner_)/2; }
	double width() const    { return max_corner_[0] - min_corner_[0]; }
	double height() const   { return max_corner_[1] - min_corner_[1]; }
	double depth() const    { return max_corner_[2] - min_corner_[2]; }

	void translate(const cml::vector3d &t)
	{
		translate(t[0], t[1], t[2]);
	}

	void translate(double x, double y, double z)
	{
		min_corner_[0] += x;
		min_corner_[1] += y;
		min_corner_[2] += z;

		max_corner_[0] += x;
		max_corner_[1] += y;
		max_corner_[2] += z;
	}

protected:
	cml::vector3d min_corner_;
	cml::vector3d max_corner_;
};

};

