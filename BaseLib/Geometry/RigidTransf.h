#pragma once

#include "BaseLib/CmlExt/CmlExt.h"


namespace mg
{
class RigidTransf
{
public:
	RigidTransf();
	RigidTransf(const RigidTransf& T);
	RigidTransf(cml::vector3d t, cml::quaterniond r);



	RigidTransf& operator=(const RigidTransf& a)
	{
		t_ = a.t_;
		r_ = a.r_;
		return *this;
	}

	RigidTransf operator*(const RigidTransf& b)
	{
		RigidTransf &a = *this;
		RigidTransf c;
		c.r_ = a.r_ * b.r_;
		c.t_ = a.t_ + cml::Rotate(a.r_, b.t_);
		return c;
	}

	static RigidTransf Lerp(const RigidTransf& a, const RigidTransf& b, double t)
	{
		return RigidTransf(cml::lerp(a.t_, b.t_, t), cml::slerp(a.r_, b.r_, t));
	}
	void Inverse();


	cml::matrix44d GetMat44() const;
	RigidTransf GetInverse() const;

	void t(cml::vector3d t) { t_=t; }
	cml::vector3d t() const { return t_; }
	cml::vector3d& t() { return t_; }

	void r(cml::quaterniond r) { r_=r; }
	cml::quaterniond r() const { return r_; }
	cml::quaterniond& r() { return r_; }

protected:
	cml::vector3d t_;
	cml::quaterniond r_;
};

}
