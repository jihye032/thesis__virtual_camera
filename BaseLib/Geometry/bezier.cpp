
#include "BaseLib/Geometry/bezier.h"

namespace mg
{

BezierCurver3d::BezierCurver3d()
{
}

BezierCurver3d::BezierCurver3d(const cml::vector3d &p1, const cml::vector3d &p2, const cml::vector3d &p3, const cml::vector3d &p4)
{
	SetControlPoints(p1, p2, p3, p4);
}

void 
BezierCurver3d::SetControlPoints(const cml::vector3d &p1, const cml::vector3d &p2, const cml::vector3d &p3, const cml::vector3d &p4)
{
	ps_[0] = p1;
	ps_[1] = p2;
	ps_[2] = p3;
	ps_[3] = p4;
}




cml::vector3d 
BezierCurver3d::Evalutate(double t) const
{
	return Evalutate(ps_[0], ps_[1], ps_[2], ps_[3], t);
}



cml::vector3d 
BezierCurver3d::Evalutate(const cml::vector3d& p1,  const cml::vector3d& p2, const cml::vector3d& p3, const cml::vector3d& p4, double t)
{
	cml::vector3d p;
	double t2 = 1-t;

	
	p[0] = t2 * ( t2*(t2*p1[0] + t*p2[0]) + t*(t2*p2[0] + t*p3[0]) )
		+ t * ( t2*(t2*p2[0] + t*p3[0]) + t*(t2*p3[0] + t*p4[0]) );

	p[1] = t2 * ( t2*(t2*p1[1] + t*p2[1]) + t*(t2*p2[1] + t*p3[1]) ) 
		+ t * ( t2*(t2*p2[1] + t*p3[1]) + t*(t2*p3[1] + t*p4[1]) );

	p[2] = t2 * ( t2*(t2*p1[2] + t*p2[2]) + t*(t2*p2[2] + t*p3[2]) )
		+ t * ( t2*(t2*p2[2] + t*p3[2]) + t*(t2*p3[2] + t*p4[2]) );
	

	return p;

}





BezierCurverQ::BezierCurverQ()
{
}

BezierCurverQ::BezierCurverQ(const cml::quaterniond& q1,  const cml::quaterniond&  q2, const cml::quaterniond& q3, const cml::quaterniond& q4)
{
	SetControlPoints(q1, q2, q3, q4);
}

void 
BezierCurverQ::SetControlPoints(const cml::quaterniond& q1,  const cml::quaterniond&  q2, const cml::quaterniond& q3, const cml::quaterniond& q4)
{
	qs_[0] = q1;
	qs_[1] = q2;
	qs_[2] = q3;
	qs_[3] = q4;
}




cml::quaterniond 
BezierCurverQ::Evalutate(double t) const
{
	return Evalutate(qs_[0], qs_[1], qs_[2], qs_[3], t);
}



cml::quaterniond 
BezierCurverQ::Evalutate(const cml::quaterniond& q1,  const cml::quaterniond&  q2, const cml::quaterniond& q3, const cml::quaterniond& q4, double t)
{
	cml::quaterniond q;
	double t2 = 1-t;

	cml::quaterniond tmpQ = cml::slerp(q2, q3, t);

	q = cml::slerp( 

		cml::slerp( cml::slerp(q1, q2, t), tmpQ, t ),
		cml::slerp( tmpQ, cml::slerp(q3, q4, t), t ),
		t

	);

	return q;

}

};