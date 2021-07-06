
#ifndef CATMULL_ROM_3F_H
#define CATMULL_ROM_3F_H


#include "BaseLib/CmlExt/CmlExt.h"
#include "BaseLib/Geometry/bezier.h"
#include "BaseLib/Geometry/RigidTransf.h"

#include <vector>

namespace mg
{

class CatmullRom3d
{
public:
	CatmullRom3d();

	void Clear();

	/*
	@param t is time.
	@param p is the desired position at time t.
	*/
	void SetKey(int index, double time, cml::vector3d p);

	/*
	@param t is time.
	@param p is the desired position at time t.
	*/
	void AddKey(double time, cml::vector3d p, bool const_vel=false, cml::vector3d v=cml::vector3d(0, 0, 0));

	void SetStartingVelocity(cml::vector3d v);
	void SetEndingVelocity(cml::vector3d v);

	void Build();

	int CountKeys() const;

	cml::vector3d Get(double t) const;

	void Sample(double time_step, std::vector<cml::vector3d> &out) const;


private:
	std::vector< double > key_times_;
	std::vector< cml::vector3d > key_poss_;
	std::vector< bool > key_const_vels_;
	std::vector< cml::vector3d > key_vels_;

	cml::vector3d starting_velocity_;
	cml::vector3d ending_velocity_;

	std::vector< BezierCurver3d > beziers_;
};



class CatmullRomQ
{
public:
	CatmullRomQ();

	void Clear();

	/*
	@param t is time.
	@param q is the desired oritentation at time t.
	*/
	void SetKey(int index, double time, cml::quaterniond q);

	/*
	@param t is time.
	@param q is the desired oritentation at time t.
	*/
	void AddKey(double time, cml::quaterniond q);

	void Build();

	int CountKeys() const;

	cml::quaterniond Get(double t) const;

	void Sample(double time_step, std::vector<cml::quaterniond> &out) const;


private:
	std::vector< double > key_times_;
	std::vector< cml::quaterniond > key_oris_;

	std::vector< BezierCurverQ > beziers_;
};



class CatmullRomRigidT
{
public:
	CatmullRomRigidT();

	void Clear();

	/*
	@param t is time.
	*/
	void SetKey(int index, double time, RigidTransf r);

	/*
	@param t is time.
	@param p is the desired position at time t.
	*/
	void AddKey(double time, RigidTransf r, bool const_vel=false, cml::vector3d v=cml::vector3d(0, 0, 0));

	void Build();

	int CountKeys() const;

	RigidTransf Get(double t) const;

	void Sample(double time_step, std::vector<RigidTransf> &out) const;


private:
	std::vector< double > key_times_;
	std::vector< RigidTransf > key_rts_;

	std::vector< bool > key_const_vels_;
	std::vector< cml::vector3d > key_vels_;

	CatmullRom3d p_spline_;
	CatmullRomQ q_spline_;
};



};

#endif