
#include "BaseLib/Geometry/CatmullRomSpline.h"
#include "BaseLib/Geometry/bezier.h"
#include "BaseLib/Motion/ccml.h"

namespace mg
{

CatmullRom3d::CatmullRom3d()
{
	starting_velocity_ = cml::vector3d(0, 0, 0);
	ending_velocity_ = cml::vector3d(0, 0, 0);
}

void CatmullRom3d::Clear()
{
	key_times_.clear();
	key_poss_.clear();
	beziers_.clear();
	key_const_vels_.clear();
	key_vels_.clear();
}


void CatmullRom3d::SetKey(int index, double time, cml::vector3d p)
{
	if ( index >= (int)key_times_.size() )
	{
		key_times_.resize(index+1);
		key_poss_.resize(index+1);
		key_const_vels_.resize(index+1);
		key_vels_.resize(index+1);
	}


	key_times_[index] = time;
	key_poss_[index] = p;
	key_const_vels_[index] = false;
}



void CatmullRom3d::AddKey(double time, cml::vector3d p, bool const_vel, cml::vector3d v)
{
	bool inserted = false;

	for ( int i=0; i<(int)key_times_.size(); i++ )
	{
		if ( time < key_times_[i] )
		{
			key_times_.insert(key_times_.begin()+i, time);
			key_poss_.insert(key_poss_.begin()+i, p);
			key_const_vels_.insert(key_const_vels_.begin()+i, const_vel);
			key_vels_.insert(key_vels_.begin()+i, v);
			inserted = true;
			break;
		}
	}

	if ( inserted == false ) 
	{
		key_times_.push_back(time);
		key_poss_.push_back(p);
		key_const_vels_.push_back(const_vel);
		key_vels_.push_back(v);
	}
}



int CatmullRom3d::CountKeys() const
{
	return (int)key_times_.size();
}

void CatmullRom3d::Build()
{
	if ( (int)key_times_.size() <=2 ) return;

	beziers_.resize(key_times_.size()-1);

	for ( int i=0; i<(int)beziers_.size(); i++ )
	{
		cml::vector3d c[4];
		c[0] = key_poss_[i];
		c[3] = key_poss_[i+1];

		cml::vector3d v1;
		cml::vector3d v2;

		if ( i==0 )
		{
			v1 = (key_poss_[1] - key_poss_[0]);
			v2 = (1.0/2.0) * (key_poss_[0] - key_poss_[2]);
		}
		else if ( i+2>=(int)key_poss_.size() )
		{
			v1 = (1.0/2.0) * (key_poss_[i+1] - key_poss_[i-1]);
			v2 = (key_poss_[i] - key_poss_[i+1]);
		}
		else
		{
			v1 = (1.0/2.0) * (key_poss_[i+1] - key_poss_[i-1]);
			v2 = (1.0/2.0) * (key_poss_[i] - key_poss_[i+2]);
		}

		if ( key_const_vels_[i] ) v1 = key_vels_[i];
		if ( key_const_vels_[i+1] ) v2 = -key_vels_[i+1];


		c[1] = c[0] + (1.0/3.0) * v1;
		c[2] = c[3] + (1.0/3.0) * v2;


		beziers_[i].SetControlPoints(c[0], c[1], c[2], c[3]);
	}
}


cml::vector3d 
CatmullRom3d::Get(double t) const
{
	if  ( key_times_.empty() ) return cml::vector3d(0, 0, 0);
	if ( (int)key_times_.size() == 1 ) return key_poss_[0];
	if ( (int)key_times_.size() == 2 ) return cml::lerp(key_poss_[0], key_poss_[1], t/(key_times_[1]-key_times_[0]));


	for ( int i=1; i<(int)key_times_.size(); i++ )
	{
		if ( t < key_times_[i] )  return beziers_[i-1].Evalutate( (t-key_times_[i-1])/(key_times_[i]-key_times_[i-1]) );
	}

	return beziers_.back().Evalutate(t-key_times_[key_times_.size()-2]);
}

void 
CatmullRom3d::Sample(double time_step, std::vector<cml::vector3d> &out) const
{
	for ( double t=0.0; t<=key_times_.back(); t+=time_step )
	{
		out.push_back(Get(t));
	}
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



CatmullRomQ::CatmullRomQ()
{
}

void CatmullRomQ::Clear()
{
	key_times_.clear();
	key_oris_.clear();
	beziers_.clear();
}


void CatmullRomQ::SetKey(int index, double time, cml::quaterniond q)
{
	if ( index >= (int)key_times_.size() )
	{
		key_times_.resize(index+1);
		key_oris_.resize(index+1);
	}


	key_times_[index] = time;
	key_oris_[index] = q;
}



void CatmullRomQ::AddKey(double time, cml::quaterniond q)
{
	bool inserted = false;

	for ( int i=0; i<(int)key_times_.size(); i++ )
	{
		if ( time < key_times_[i] )
		{
			key_times_.insert(key_times_.begin()+i, time);
			key_oris_.insert(key_oris_.begin()+i, q);
			inserted = true;
			break;
		}
	}

	if ( inserted == false ) 
	{
		key_times_.push_back(time);
		key_oris_.push_back(q);
	}
}



int CatmullRomQ::CountKeys() const
{
	return (int)key_times_.size();
}

void CatmullRomQ::Build()
{
	if ( (int)key_times_.size() <=2 ) return;

	beziers_.resize(key_times_.size()-1);

	for ( int i=0; i<(int)beziers_.size(); i++ )
	{
		cml::quaterniond c[4];
		c[0] = key_oris_[i];
		c[3] = key_oris_[i+1];

		if ( i==0 )
		{
			c[1] = EXP((1.0/3.0) * LOG(key_oris_[1] * cml::inverse(key_oris_[0])) ) * c[0];
			c[2] = EXP((1.0/6.0) * LOG(key_oris_[i] * cml::inverse(key_oris_[i+2])) ) * c[3];
		}
		else if ( i+2>=(int)key_oris_.size() )
		{
			c[1] = EXP((1.0/6.0) * LOG(key_oris_[i+1] * cml::inverse(key_oris_[i-1])) ) * c[0];
			c[2] = EXP((1.0/3.0) * LOG(key_oris_[i] * cml::inverse(key_oris_[i+1])) ) * c[3];
		}
		else
		{
			c[1] = EXP((1.0/6.0) * LOG(key_oris_[i+1] * cml::inverse(key_oris_[i-1])) ) * c[0];
			c[2] = EXP((1.0/6.0) * LOG(key_oris_[i] * cml::inverse(key_oris_[i+2])) ) * c[3];
		}


		beziers_[i].SetControlPoints(c[0], c[1], c[2], c[3]);
	}
}


cml::quaterniond 
CatmullRomQ::Get(double t) const
{
	if  ( key_times_.empty() ) return cml::quaterniond(1, 0, 0, 0);
	if ( (int)key_times_.size() == 1 ) return key_oris_[0];
	if ( (int)key_times_.size() == 2 ) return cml::slerp(key_oris_[0], key_oris_[1], t/(key_times_[1]-key_times_[0]));


	for ( int i=1; i<(int)key_times_.size(); i++ )
	{
		if ( t < key_times_[i] )  return beziers_[i-1].Evalutate( (t-key_times_[i-1])/(key_times_[i]-key_times_[i-1]) );
	}

	return beziers_.back().Evalutate(t-key_times_[key_times_.size()-2]);
}


void 
CatmullRomQ::Sample(double time_step, std::vector<cml::quaterniond> &out) const
{
	for ( double t=0.0; t<=key_times_.back(); t+=time_step )
	{
		out.push_back(Get(t));
	}
}




///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



CatmullRomRigidT::CatmullRomRigidT()
{
}

void CatmullRomRigidT::Clear()
{
	key_times_.clear();
	key_rts_.clear();
	p_spline_.Clear();
	q_spline_.Clear();
	key_const_vels_.clear();
	key_vels_.clear();
}


void CatmullRomRigidT::SetKey(int index, double time, RigidTransf r)
{
	if ( index >= (int)key_times_.size() )
	{
		key_times_.resize(index+1);
		key_rts_.resize(index+1);
		key_const_vels_.resize(index+1);
		key_vels_.resize(index+1);

	}


	key_const_vels_[index] = false;
	key_times_[index] = time;
	key_rts_[index] = r;
}



void CatmullRomRigidT::AddKey(double time, RigidTransf r, bool const_vel, cml::vector3d v)
{
	bool inserted = false;

	for ( int i=0; i<(int)key_times_.size(); i++ )
	{
		if ( time < key_times_[i] )
		{
			key_times_.insert(key_times_.begin()+i, time);
			key_rts_.insert(key_rts_.begin()+i, r);
			key_const_vels_.insert(key_const_vels_.begin()+i, const_vel);
			key_vels_.insert(key_vels_.begin()+i, v);
			inserted = true;
			break;
		}
	}

	if ( inserted == false ) 
	{
		key_times_.push_back(time);
		key_rts_.push_back(r);
		key_const_vels_.push_back(const_vel);
		key_vels_.push_back(v);
	}
}



int CatmullRomRigidT::CountKeys() const
{
	return (int)key_times_.size();
}

void CatmullRomRigidT::Build()
{
	if ( (int)key_times_.size() <=2 ) return;

	p_spline_.Clear();
	q_spline_.Clear();
	for ( int i=0; i<(int)key_times_.size(); i++ )
	{
		p_spline_.AddKey(key_times_[i], key_rts_[i].t(), key_const_vels_[i], key_vels_[i]);
		q_spline_.AddKey(key_times_[i], key_rts_[i].r());
	}

	p_spline_.Build();
	q_spline_.Build();
}


RigidTransf 
CatmullRomRigidT::Get(double t) const
{
	if  ( key_times_.empty() ) return RigidTransf();
	if ( (int)key_times_.size() == 1 ) return key_rts_[0];
	if ( (int)key_times_.size() == 2 ) return RigidTransf::Lerp(key_rts_[0], key_rts_[1], t);


	return RigidTransf(p_spline_.Get(t), q_spline_.Get(t));
}


void 
CatmullRomRigidT::Sample(double time_step, std::vector<RigidTransf> &out) const
{
	double t;
	for ( t=0.0; t<=key_times_.back(); t+=time_step )
	{
		out.push_back(Get(t));
	}
}


};