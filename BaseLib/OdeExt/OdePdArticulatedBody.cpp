#ifdef ODE_EXT


#include "BaseLib/OdeExt/OdePdArticulatedBody.h"


void 
OdePdArticulatedBody::InitializePdControls()
{
	pd_controls_.clear();
	pd_controls_.resize( ode_joints_.size() );

	for ( unsigned int i=0; i<pd_controls_.size(); i++ )
	{
		if ( ode_joints_[i] != 0 )
		{
			if ( ode_joints_[i]->joint_type() == dJointTypeHinge )
			{
				pd_controls_[i].On();
				//pd_controls_[i].input(ode_joints_[i]->d_hinge()->getAngle());
				//pd_controls_[i].target(ode_joints_[i]->d_hinge()->getAngle());
			}
		}
	}
}

void
OdePdArticulatedBody::SetPdTargetPose(const UrdfPose &taget_pose)
{
	for ( unsigned int i=0; i<pd_controls_.size(); i++ )
	{
		if ( ode_joints_[i] != 0 )
		{
			if ( ode_joints_[i]->joint_type() == dJointTypeHinge )
			{
				pd_controls_[i].target(taget_pose.angle(i));
			}
		}
	}
}

void
OdePdArticulatedBody::StepPD(double elipsed_time)
{
	for ( unsigned int i=0; i<pd_controls_.size(); i++ )
	{
		if ( ode_joints_[i] != 0 )
		{
			if ( ode_joints_[i]->joint_type() == dJointTypeHinge )
			{
				
				pd_controls_[i].input(ode_joints_[i]->d_hinge()->getAngle());
				pd_controls_[i].Step(elipsed_time);
			
				//ode_joints_[i]->d_hinge()->setParam(dMax);
				ode_joints_[i]->d_hinge()->addTorque(pd_controls_[i].output());

				
			}
		}

	}
}

#endif
