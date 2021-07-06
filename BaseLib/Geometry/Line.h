
#pragma once


#include "BaseLib/CmlExt/CmlExt.h"

namespace mg
{

class LineSegment;

class Line
{
public:
	Line();
	Line(const cml::vector3d &a_point, const cml::vector3d &dir);

	void DefineByPointAndDir(const cml::vector3d &a_point, const cml::vector3d &dir);
	void DefineByTwoPoints(const cml::vector3d &a_point, const cml::vector3d &b_point);

	bool IsParallelWith(const Line &line_B) const;
	cml::vector3d FootOfPerpendicular(const cml::vector3d &from) const;
	double MinDistToPoint(const cml::vector3d &p) const;
	double MinDistToLine(const Line &line_B) const;
	LineSegment BridgeToLine(const Line &line_b) const; /// shortest LineSegment connecting two lines. 

	
	void dir(const cml::vector3d &d) { dir_ = d; }
	void point(const cml::vector3d &p) { point_ = p; }
	cml::vector3d dir() const { return dir_; }
	cml::vector3d point() const { return point_; }

protected:
	cml::vector3d point_;
	cml::vector3d dir_;
};


class HalfLine
{
public:
	HalfLine(){};
	HalfLine(const cml::vector3d &point, const cml::vector3d &dir);
	void DefineByPointAndDir(const cml::vector3d &a_point, const cml::vector3d &dir);

	double MinDistToPoint(const cml::vector3d &p) const;


	void dir(const cml::vector3d &d) { dir_ = d; }
	void point(const cml::vector3d &p) { point_ = p; }
	cml::vector3d dir() const { return dir_; }
	cml::vector3d point() const { return point_; }

protected:
	cml::vector3d point_;
	cml::vector3d dir_;
};



class LineSegment
{
public:
	LineSegment() {}
	LineSegment(const cml::vector3d &point_a, const cml::vector3d &point_b);

	void points(const cml::vector3d &point_a, const cml::vector3d &point_b);
	std::pair< cml::vector3d, cml::vector3d> points() const;
	void point_A(const cml::vector3d &a) { point_A_ = a; }
	void point_B(const cml::vector3d &b) { point_B_ = b; }
	cml::vector3d point_A() const { return point_A_; }
	cml::vector3d point_B() const { return point_B_; }
	
	cml::vector3d FootOfPerpendicular(const cml::vector3d &from) const;
	double MinDistToPoint(const cml::vector3d &p) const;
	LineSegment BridgeToLineSeg(const LineSegment &line_b) const; /// shortest LineSegment connecting two lines. 
	LineSegment BridgeToPoint(const cml::vector3d &pos) const; /// shortest LineSegment connecting with the point. 

	double length() const;
	cml::vector3d lowerPoint() const { return (point_A_[1]<point_B_[1]) ? point_A_ : point_B_; }
	cml::vector3d higherPoint() const { return (point_A_[1]>point_B_[1]) ? point_A_ : point_B_; }

	void Translate(cml::vector3d v);
	void Rotate(cml::quaterniond q);
	void Scale(cml::vector3d s);

protected:
	cml::vector3d point_A_;
	cml::vector3d point_B_;
};


}

