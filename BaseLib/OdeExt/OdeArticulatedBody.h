#ifdef ODE_EXT

#pragma once

#include "BaseLib/OdeExt/OdeExt.h"
#include "BaseLib/OdeExt/URDFMotion.h"
#include "BaseLib/Geometry/PrimitiveShape.h"

class PmHuman;
class PmPosture;

class OdeArticulatedBody
{
public:
	OdeArticulatedBody();
	OdeArticulatedBody(dWorldID dworld_id, dSpaceID dspace_id);
	~OdeArticulatedBody();
	void Clear();

	void SetOdeWorldSpace(dWorldID dworld_id, dSpaceID dspace_id);

	OdeBodyExt* AddNewBody();
	OdeJointExt* AddNewHingeJoint();
	OdeJointExt* AddNewFixedJoint();

	void SetCurrentStateAsInit();
	void SetPose(const UrdfPose &pose);

	void Create(Urdf &urdf);
	void Create(std::string urdf_file);
	void Create(PmPosture *base_pose);
	void Create(PmHuman *base_body);

	void ResetDynamics();
	void DrawGL();

protected:
	void ClearBodies();
	void ClearJoints();


protected:
	dWorldID dworld_id_;
	dSpaceID dspace_id_;

	OdeBodyGroup ode_bodies_;
	OdeJointGroup ode_joints_;

};


#endif






