#include <math.h>
#include "BaseLib/Geometry/TRS_Transf.h"


namespace mg
{

TRS_Transf::TRS_Transf()
{
	SetIndentity();
}
	
TRS_Transf::TRS_Transf(const TRS_Transf& T)
{
	t_ = T.t_;
	r_ = T.r_;
	s_ = T.s_;
}

	TRS_Transf::TRS_Transf(cml::vector3d t, cml::quaterniond r, cml::vector3d s)
{
	t_ = t;
	r_ = r;
	s_ = s;
}

void
TRS_Transf::SetIndentity()
{
	t_.zero();
	r_.identity();
	s_.set(1., 1., 1.);
}


void
TRS_Transf::Scale(double x, double y, double z)
{
	Scale(cml::vector3d(x, y, z));
}

void
TRS_Transf::Scale(cml::vector3d &s)
{
	t_ = s*t_;
	s_ = s*s_;
}

void
TRS_Transf::Translate(double x, double y, double z)
{
	Translate(cml::vector3d(x, y, z));
}

void
TRS_Transf::Translate(cml::vector3d &t)
{
	t_ = t + t_;
}

void
TRS_Transf::Rotate(cml::quaterniond &q)
{
	r_ = q * r_;
	t_ = cml::Rotate(q, t_);
}

void
TRS_Transf::Rotate(cml::vector3d &axis, double angle)
{
	cml::quaterniond q;
	cml::quaternion_rotation_axis_angle(q, axis, angle);
	Rotate(q);
}

void 
TRS_Transf::MultiMat44(cml::matrix44d &mat) 
{
	TRS_Transf m;
	m.SetMat44(mat);

	*this = *this * m;
}

void 
TRS_Transf::SetMat44(cml::matrix44d &mat) 
{
	cml::TRSdecompose(mat, t_, r_, s_);
}

cml::matrix44d 
TRS_Transf::GetMat44() const
{
	return cml::MatrixTranslation(t_) * cml::MatrixRotationQuaternion(r_) * cml::MatrixScaling(s_);
}

void
TRS_Transf::Inverse()
{
	cml::TRSdecompose(cml::inverse(GetMat44()), t_, r_, s_);
}


TRS_Transf
TRS_Transf::GetInverse() const
{
	TRS_Transf trs = *this;
	trs.Inverse();
	return trs;
}


std::string
TRS_Transf::ToString() const
{
	std::stringstream sstr;
	sstr << "T: " << t_ << ", Q: " << r_ << ", S: " << s_;
	return sstr.str();
}

cml::vector3d operator*(const TRS_Transf & t, const cml::vector3d & b)
{
	return cml::Rotate(t.r_, t.s_*b) + t.t_;
}

};