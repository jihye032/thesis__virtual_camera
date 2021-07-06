
#ifndef BEZIER_H
#define BEZIER_H


#include "BaseLib/CmlExt/CmlExt.h"


namespace mg
{

cml::quaterniond bezier(cml::quaterniond& q1,  cml::quaterniond&  q2, cml::quaterniond& q3, cml::quaterniond& q4,  float& t);




class BezierCurver3d
{
public:
	BezierCurver3d();
	BezierCurver3d(const cml::vector3d &p1, const cml::vector3d &p2, const cml::vector3d &p3, const cml::vector3d &p4);

	void SetControlPoints(const cml::vector3d &p1, const cml::vector3d &p2, const cml::vector3d &p3, const cml::vector3d &p4);

	cml::vector3d Evalutate(double t) const;

	static cml::vector3d Evalutate(const cml::vector3d& p1,  const cml::vector3d&  p2, const cml::vector3d& p3, const cml::vector3d& p4,  double t);

protected:
	cml::vector3d ps_[4];

};

class BezierCurverQ
{
public:
	BezierCurverQ();
	BezierCurverQ(const cml::quaterniond& q1,  const cml::quaterniond&  q2, const cml::quaterniond& q3, const cml::quaterniond& q4);

	void SetControlPoints(const cml::quaterniond& q1,  const cml::quaterniond&  q2, const cml::quaterniond& q3, const cml::quaterniond& q4);

	cml::quaterniond Evalutate(double t) const;

	static cml::quaterniond Evalutate(const cml::quaterniond& q1,  const cml::quaterniond&  q2, const cml::quaterniond& q3, const cml::quaterniond& q4,  double t);

protected:
	cml::quaterniond qs_[4];

};

};
#endif