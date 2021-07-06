#ifdef ODE_EXT

#pragma once

#include <ode/ode.h>
#include "BaseLib/CmlExt/CmlExt.h"
#include <vector>
#include "BaseLib/Geometry/PrimitiveShape.h"

class OdeBodyExt;

class OdeBodyDynamicState
{
	friend class OdeBodyExt;

public:
	void Set(const OdeBodyExt &body);

	cml::vector3d position() const { return dRealToVector(p_); }
	cml::quaterniond orientation() const { return dRealToQuater(q_); }
	cml::vector3d linear_vel() const { return dRealToVector(linear_v_); }
	cml::vector3d angular_vel() const { return dRealToVector(angular_v_); }
	cml::vector3d force() const { return dRealToVector(force_); }
	cml::vector3d torque() const { return dRealToVector(torque_); }

	const dReal* d_position() const { return p_; }
	const dReal* d_orientation() const { return q_; }
	const dReal* d_linear_vel() const { return linear_v_; }
	const dReal* d_angular_vel() const { return angular_v_; }
	const dReal* d_force() const { return force_; }
	const dReal* d_torque() const { return torque_; }

protected:
	inline cml::vector3d dRealToVector(const dReal* d) const;
	inline cml::quaterniond dRealToQuater(const dReal *d) const;

	dVector3 p_;
	dQuaternion q_;
	dVector3 linear_v_;
	dVector3 angular_v_;
	dVector3 force_;
	dVector3 torque_;
};

class OdeBodyExt : public dBody
{
public:
	OdeBodyExt(dWorldID dworld_id);
	virtual ~OdeBodyExt();

	void CreateGeometry(dSpaceID space_id, mg::PrimitiveShape* s, double mass);
	void CreateGeometries(dSpaceID space_id, const std::vector<mg::PrimitiveShape*> &shapes, const std::vector<double> &masses);
	void DeleteGeometries();

	void SetCurrentAsInitialDynamics();
	void ResetDynamics();

	void SetOrietationExt(cml::quaterniond q);
	void SetPositionExt(cml::vector3d p);
	void SetOffestPosition(cml::vector3d p);

	cml::quaterniond orietation() const { const dReal* q=this->getQuaternion(); return cml::quaterniond(q[0], q[1], q[2], q[3]); }
	cml::vector3d position() const { const dReal* p=this->getPosition(); return cml::vector3d(p[0], p[1], p[2]); }

protected:
	cml::vector3d dRealToVector(const dReal* d) const;
	cml::quaterniond dRealToQuater(const dReal *d) const;
	void vectorTodVector3(const cml::vector3d &d, dVector3 out) const;
	void quaterTodQuater(const cml::quaterniond &d, dQuaternion out) const;

protected:
	OdeBodyDynamicState init_dynamic_state_;
	cml::vector3d offset_position_;
	cml::vector3d global_position_;
};


class OdeBodyGroup : public std::vector<OdeBodyExt*>
{
public:

	void Disable();
	void Enable();
	void SetGravityMode(bool mode);
	void SetCurrentAsInitialDynamics();
	void ResetDynamics();
};


class OdeJointExt
{
public:
	OdeJointExt(dJoint *j);
	virtual ~OdeJointExt();

	int joint_type() const { return d_joint_->getType(); }

	dBallJoint* d_ball() { return (dBallJoint*)d_joint_; }
	dHingeJoint* d_hinge() { return (dHingeJoint*)d_joint_; }
	dFixedJoint* d_fixed() { return (dFixedJoint*)d_joint_; }
	dAMotorJoint* d_amotor() { return (dAMotorJoint*)d_joint_; }

	dJoint *d_joint_;
};

class OdeJointGroup : public std::vector<OdeJointExt*>
{
public:

};


/**
This function doesn't consider the transformation of the input_primitive.
The created dGeom always will be located at the origin.
It the case of cylinder or capsule, it assumes the it is laid on the z axis.
Thus, input_primitive->direction() should be 2.
*/
dGeomID 
OdeCreateDGeom(dSpaceID dspace_id, const mg::PrimitiveShape *input_primitive);


/**
This function doesn't consider the transformation of the input_primitive.
The created dGeom always will be located at the origin.
It the case of cylinder or capsule, it assumes the it is laid on the z axis.
Thus, input_primitive->direction() should be 2.
*/
dGeomID 
OdeCreateDGeom(dSpaceID dspace_id, const mg::PrimitiveShape *input_primitive, double in_density, dMass &out_mass);


cml::vector3d ode2cml(const dVector3 v);



#endif
