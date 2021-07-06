#include <math.h>
#include "BaseLib/Geometry/RigidTransf.h"


namespace mg
{

RigidTransf::RigidTransf()
{
	t_.set(0, 0, 0);
	r_.identity();
}
	
RigidTransf::RigidTransf(const RigidTransf& T)
{
	t_ = T.t_;
	r_ = T.r_;
}

RigidTransf::RigidTransf(cml::vector3d t, cml::quaterniond r)
{
	t_ = t;
	r_ = r;
}


cml::matrix44d 
RigidTransf::GetMat44() const
{
	return cml::MatrixTranslation(t_) * cml::MatrixRotationQuaternion(r_);
}


void
RigidTransf::Inverse()
{
	r_.inverse();
	t_ = cml::Rotate(r_, -1*t_);
}

RigidTransf 
RigidTransf::GetInverse() const
{
	RigidTransf b(*this);
	b.Inverse();
	return b;
}


};