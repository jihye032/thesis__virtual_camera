#ifdef ODE_EXT


#include "BaseLib/OdeExt/OdeArticulatedBody.h"
#include "BaseLib/OdeExt/OdeGL.h"


OdeArticulatedBody::OdeArticulatedBody()
{
	dworld_id_ = 0;
	dspace_id_ = 0;
}

OdeArticulatedBody::OdeArticulatedBody(dWorldID dworld_id, dSpaceID dspace_id)
{
	SetOdeWorldSpace(dworld_id, dspace_id);
}

OdeArticulatedBody::~OdeArticulatedBody()
{
	Clear();
}

void
OdeArticulatedBody::SetOdeWorldSpace(dWorldID dworld_id, dSpaceID dspace_id)
{
	dworld_id_ = dworld_id;
	dspace_id_ = dspace_id;
}


void
OdeArticulatedBody::Clear()
{
	ClearBodies();
	ClearJoints();

	dworld_id_ = 0;
	dspace_id_ = 0;
}

void
OdeArticulatedBody::ClearBodies()
{
	for ( unsigned int i=0; i<ode_bodies_.size(); i++ )
	{
		if ( ode_bodies_[i] != 0 )
			delete ode_bodies_[i];
	}
	ode_bodies_.clear();
}

void
OdeArticulatedBody::ClearJoints()
{
	for ( unsigned int i=0; i<ode_joints_.size(); i++ )
	{
		if ( ode_joints_[i] != 0 )
			delete ode_joints_[i];
	}
	ode_joints_.clear();


}


OdeBodyExt* 
OdeArticulatedBody::AddNewBody()
{
	OdeBodyExt *b = new OdeBodyExt(dworld_id_);

	ode_bodies_.push_back(b);

	return b;
}

OdeJointExt* 
OdeArticulatedBody::AddNewHingeJoint()
{
	OdeJointExt *j = new OdeJointExt(new dHingeJoint(dworld_id_, 0));
	ode_joints_.push_back(j);

	return j;
}

OdeJointExt* 
OdeArticulatedBody::AddNewFixedJoint()
{
	OdeJointExt *j = new OdeJointExt(new dFixedJoint(dworld_id_, 0));
	ode_joints_.push_back(j);

	return j;
}

void 
OdeArticulatedBody::SetCurrentStateAsInit()
{
	ode_bodies_.SetCurrentAsInitialDynamics();
}


void
OdeArticulatedBody::SetPose(const UrdfPose &pose)
{
	const Urdf *urdf = pose.urdf();

	for ( int i=0; i<urdf->num_links(); i++ )
	{
		if ( urdf->link(i) == 0 ) continue;
		if ( (int)ode_bodies_.size() <= i ) continue;
		if ( ode_bodies_[i] == 0 ) continue;
		if ( urdf->joint(urdf->link(i)->parent_joint_name_) == 0 ) continue;
		
		const std::string &joint_name = urdf->link(i)->parent_joint_name_;


		ode_bodies_[i]->SetOrietationExt(pose.GetGlobalJointRotation(joint_name));
		ode_bodies_[i]->SetPositionExt(pose.GetGlobalJointPosition(joint_name));
	}


}



void
OdeArticulatedBody::DrawGL()
{
	for ( unsigned int i=0; i<ode_bodies_.size(); i++ )
	{
		DrawOdeBody(ode_bodies_[i]->id());
	}

}

void
OdeArticulatedBody::ResetDynamics()
{
	ode_bodies_.ResetDynamics();
}


#endif