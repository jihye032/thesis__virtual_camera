#pragma once

#include "BaseLib/CmlExt/CmlExt.h"


namespace mg
{

/**
TRS_Transf is combination of three transformations, translaition, rotataion ans scaling. 
TRS = Translation * Rotation * Scaling. 
*/
class TRS_Transf
{
public:
	TRS_Transf();
	TRS_Transf(const TRS_Transf& T);
	TRS_Transf(cml::vector3d t, cml::quaterniond r, cml::vector3d s);

	void SetIndentity();

	TRS_Transf& operator=(const TRS_Transf& a)
	{
		t_ = a.t_;
		r_ = a.r_;
		s_ = a.s_;
		return *this;
	}

	TRS_Transf operator*(const TRS_Transf& b)
	{
		const TRS_Transf &a = *this;
		TRS_Transf c;
		
		// Scale
		c.s_ = a.s_ * b.s_;

		// Rotation
		c.r_ = a.r_ * b.r_;

		// Translation
		c.t_ = a.s_ * b.t_;
		c.t_ = a.t_ + cml::Rotate(a.r_, c.t_);

		return c;
	}

	/*cml::vector3d operator*(const cml::vector3d& b)
	{
		return cml::Rotate(r_, s_*b) + t_;
	}*/

	friend cml::vector3d operator*(const TRS_Transf& t, const cml::vector3d& b);

	static TRS_Transf Lerp(const TRS_Transf& a, const TRS_Transf& b, double t)
	{
		return TRS_Transf(cml::lerp(a.t_, b.t_, t), cml::slerp(a.r_, b.r_, t), cml::lerp(a.s_, b.s_, t));
	}


	void Scale(double x, double y, double z);
	void Scale(cml::vector3d &s);
	void Translate(double x, double y, double z);
	void Translate(cml::vector3d &t);
	void Rotate(cml::quaterniond &q);
	void Rotate(cml::vector3d &axis, double angle);

	
	void SetMat44(cml::matrix44d &mat);
	void MultiMat44(cml::matrix44d &mat);
	cml::matrix44d GetMat44() const;

	
	void Inverse();
	TRS_Transf GetInverse() const;

	void t(cml::vector3d t) { t_=t; }
	cml::vector3d t() const { return t_; }
	cml::vector3d& t() { return t_; }

	void r(cml::quaterniond r) { r_=r; }
	cml::quaterniond r() const { return r_; }
	cml::quaterniond& r() { return r_; }

	void s(cml::vector3d s) { s_ = s; }
	cml::vector3d s() const { return s_; }
	cml::vector3d& s() { return s_; }

	void SetUniformScale(double f) { s({f, f, f}); }

	std::string ToString() const;

protected:
	cml::vector3d t_;
	cml::quaterniond r_;
	cml::vector3d s_;
};


}




