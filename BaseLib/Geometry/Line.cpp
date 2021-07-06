
#include "BaseLib/Geometry/Line.h"
#include <vector>

namespace mg
{

/////////////////////////////////////////////////////////////////////
// Class: Line
Line::Line()
{
}

Line::Line(const cml::vector3d &point, const cml::vector3d &dir)
{
	DefineByPointAndDir(point, dir);
}

void
Line::DefineByPointAndDir(const cml::vector3d &p, const cml::vector3d &d)
{
	point(p);
	dir(d);
}

void
Line::DefineByTwoPoints(const cml::vector3d &point_a, const cml::vector3d &point_b)
{
	point(point_a);
	dir(normalize(point_b-point_a));
}


cml::vector3d
Line::FootOfPerpendicular(const cml::vector3d &from) const
{
	cml::vector3d p_to_from = from - point_;

	return point_ + dir_ * ( (cml::dot(p_to_from, dir_) / cml::length(dir_)) );
}

double
Line::MinDistToPoint(const cml::vector3d &p) const
{
	return cml::length( p - FootOfPerpendicular(p) );
}


bool
Line::IsParallelWith(const Line &line_B) const
{
	if ( cml::length(cml::cross(cml::normalize(line_B.dir_), cml::normalize(dir_))) < cml::EPS ) 
		return true;

	return false;
}

LineSegment 
Line::BridgeToLine(const Line &line_B) const
{
	const Line &line_A = *this;

	const cml::vectord &pA = line_A.point_;
	const cml::vectord &pB = line_B.point_;

	cml::vector3d vA = line_A.dir_;
	cml::vector3d vB = line_B.dir_;
	cml::vector3d vC = cml::cross(vA,vB);	// vC is the vector perpendicular to both vA and vB.

	// if vA and vB are parallel.
	if ( cml::length(vC) < 0.1 )
	{
		// Arbitrary points on the lines.
		cml::vector3d any_pA = pA - 10 * vA;
		cml::vector3d any_pB = pB - 100 * vB;

		cml::vector3d d = cml::cross(vA, (any_pB-any_pA));
		vC =cml::cross( d , vA);
	}

	vC = normalize(vC);


	//////////////////////////////////////////////
	// (pA + x[0]*vA) - (pB + x[1]*vB) = x[2]*vC;
	//
	// The line segment Q-R.
	// Q <== pA + x[0]*vA;
	// R <== pB + x[1]*vB;
	//
	// Build linear System, Ax = b.
	//
	cml::matrixd A;
	cml::vector3d x, b;
	A(0,0) = vA[0];
	A(0,1) = -1*vB[0];
	A(0,2) = -1*vC[0];
		 
	A(1,0) = vA[1];
	A(1,1) = -1*vB[1];
	A(1,2) = -1*vC[1];
		 
	A(2,0) = vA[2];
	A(2,1) = -1*vB[2];
	A(2,2) = -1*vC[2];

	b[0] = pB[0] - pA[0];
	b[1] = pB[1] - pA[1];
	b[2] = pB[2] - pA[2];

	x = A.inverse() * b;

	//return LineSegment(pA + x[0]*vA, pB + x[1]*vB);
	return LineSegment(pA + x[0]*vA, (pA + x[0]*vA) - vC*x[2]);
}


double
Line::MinDistToLine(const Line &line_B) const
{
	LineSegment ls = BridgeToLine(line_B);

	return ls.length();
}



/////////////////////////////////////////////////////////////////////
// Class: HalfLine
HalfLine::HalfLine(const cml::vector3d &point, const cml::vector3d &dir)
{
	DefineByPointAndDir(point, dir);
}

void
HalfLine::DefineByPointAndDir(const cml::vector3d &p, const cml::vector3d &d)
{
	point(p);
	dir(d);
}

double
HalfLine::MinDistToPoint(const cml::vector3d &target) const
{
	cml::vector3d end_point_to_target = target-point_;

	if ( cml::dot(dir_ , end_point_to_target) < 0 )
	{
		return cml::length(end_point_to_target);
	}
	else
	{
		Line line(point_, dir_);
		return line.MinDistToPoint(target);
	}
}

/////////////////////////////////////////////////////////////////////
// Class: LineSegment
LineSegment::LineSegment(const cml::vector3d &point_a, const cml::vector3d &point_b)
{
	points(point_a, point_b);
}


void
LineSegment::points(const cml::vector3d &point_a, const cml::vector3d &point_b)
{
	point_A(point_a);
	point_B(point_b);
}

std::pair< cml::vector3d, cml::vector3d>
LineSegment::points() const
{
	return std::make_pair(point_A_, point_B_);
}

double
LineSegment::length() const
{
	return cml::length(point_A_-point_B_);
}

cml::vector3d
LineSegment::FootOfPerpendicular(const cml::vector3d &p) const
{
	cml::vector3d A_to_P = p - point_A_;
	cml::vector3d A_to_B = point_B_ - point_A_;

	return point_A_ + A_to_B.normalize() * (cml::dot(A_to_P, A_to_B) / cml::length(A_to_B));
}

double
LineSegment::MinDistToPoint(const cml::vector3d &p) const
{
	cml::vector3d A_to_P = p-point_A_;
	cml::vector3d B_to_P = p-point_B_;
	cml::vector3d A_to_B = point_B_-point_A_;

	if ( cml::dot( A_to_B, A_to_P) < 0 )
	{
		return cml::length( A_to_P );
	}
	else if ( cml::dot((-1*A_to_B), B_to_P) < 0 )
	{
		return cml::length( B_to_P );
	}
	else
	{
		return cml::length( p - FootOfPerpendicular(p) );
	}
}


LineSegment 
LineSegment::BridgeToLineSeg(const LineSegment &line_seg_B) const
{
	const LineSegment &line_seg_A = *this;

	cml::vector3d a_p0 = line_seg_A.point_A();
	cml::vector3d a_p1 = line_seg_A.point_B();
	cml::vector3d b_p0 = line_seg_B.point_A();
	cml::vector3d b_p1 = line_seg_B.point_B();

	Line line_A(a_p0, normalize(a_p1-a_p0));
	Line line_B(b_p0, normalize(b_p1-b_p0));

	LineSegment line_bridge = line_A.BridgeToLine(line_B);
	if ( cml::dot((line_bridge.point_A()-a_p0),(a_p1-a_p0)) >= 0 
		&& cml::dot((line_bridge.point_A()-a_p1),(a_p0-a_p1)) >= 0 
		&& cml::dot((line_bridge.point_B()-b_p0),(b_p1-b_p0)) >= 0 
		&& cml::dot((line_bridge.point_B()-b_p1),(b_p0-b_p1)) >= 0 ) 
	{
		return line_bridge;
	}
	else if( cml::dot((line_bridge.point_A()-a_p0),(a_p1-a_p0)) >= 0 
		&& cml::dot((line_bridge.point_A()-a_p1),(a_p0-a_p1)) >= 0 )
	{
		if ( line_seg_A.MinDistToPoint(b_p0) < line_seg_A.MinDistToPoint(b_p1) )
		{
			return LineSegment(line_seg_A.FootOfPerpendicular(b_p0), b_p0);
		}
		else
		{
			return LineSegment(line_seg_A.FootOfPerpendicular(b_p1), b_p1);
		}
	}
	else if( cml::dot((line_bridge.point_B()-b_p0),(b_p1-b_p0)) >= 0 
		&& cml::dot((line_bridge.point_B()-b_p1),(b_p0-b_p1)) >= 0 )
	{
		if ( line_seg_B.MinDistToPoint(a_p0) < line_seg_B.MinDistToPoint(a_p1) )
		{
			return LineSegment(a_p0, line_seg_B.FootOfPerpendicular(a_p0));
		}
		else
		{
			return LineSegment(a_p1, line_seg_B.FootOfPerpendicular(a_p1));
		}
	}
	
	std::vector<LineSegment> bridges;
	bridges.resize(4);
	bridges[0].points( a_p0, b_p0 );
	bridges[1].points( a_p0, b_p1 );
	bridges[2].points( a_p1, b_p0 );
	bridges[3].points( a_p1, b_p1 );

	int min_id=0;
	double min_len = bridges[0].length();
	for ( int i=1; i<4; i++ )
	{
		if ( bridges[i].length() < min_len )
		{
			min_len = bridges[i].length();
			min_id = i;
		}
	}
	return bridges[min_id];
}


LineSegment 
LineSegment::BridgeToPoint(const cml::vector3d &p) const
{
	cml::vector3d A_to_P = p-point_A_;
	cml::vector3d B_to_P = p-point_B_;
	cml::vector3d A_to_B = point_B_-point_A_;

	if ( cml::dot( A_to_B, A_to_P) < 0 )
	{
		return LineSegment(point_A_, p);
	}
	else if ( cml::dot((-1*A_to_B) , B_to_P) < 0 )
	{
		return LineSegment(point_B_, p);
	}
	else
	{
		return LineSegment(FootOfPerpendicular(p), p);
	}
}

void
LineSegment::Translate(cml::vector3d t)
{
	point_A_ += t;
	point_B_ += t;
}

void
LineSegment::Rotate(cml::quaterniond q)
{
	point_A_ = cml::Rotate(q, point_A_);
	point_B_ = cml::Rotate(q, point_B_);
}

void
LineSegment::Scale(cml::vector3d s)
{
	for ( int i=0; i<3; i++ )
	{
		point_A_[i] *= s[i];
		point_B_[i] *= s[i];
	}
}


}
