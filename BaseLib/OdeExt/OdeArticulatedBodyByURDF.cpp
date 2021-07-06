#ifdef ODE_EXT

#include "BaseLib/OdeExt/OdeArticulatedBody.h"
#include "BaseLib/OdeExt/OdeGL.h"
#include "BaseLib/Motion/ml.h"
#include "BaseLib/PMU/PmBoundingVolume.h"

void
OdeArticulatedBody::Create(std::string urdf_file)
{
	Urdf urdf;
	urdf.LoadURDF(urdf_file);
	Create(urdf);
}

void
OdeArticulatedBody::Create(Urdf &urdf)
{
	ClearBodies();
	ClearJoints();

	/// Body
	for ( int i=0; i<urdf.num_links(); i++ )
	{
		UrdfLink *u_link = urdf.link(i);
		UrdfJoint *p_joint = urdf.joint( u_link->parent_joint_name_ );

		if ( u_link->mass_ < 10e-4 )
		{
			printf("An ode body mass was close to 0.\n");
		}
		
		/// ODE Body
		OdeBodyExt *dbody = AddNewBody();

		/// Geom & Mass
		{
			std::vector<mg::PrimitiveShape*> geoms;
			std::vector<double> masses;

			if ( u_link->bounding_volume_ != 0 )
			{
				geoms.push_back( u_link->bounding_volume_ );
				masses.push_back( u_link->mass_ );
			}

			for ( unsigned int i=0; i<u_link->other_bounding_volumes_.size(); i++ )
			{
				geoms.push_back(u_link->other_bounding_volumes_[i]);
				masses.push_back( u_link->mass_ );
			}

			for ( unsigned int i=0; i<masses.size(); i++ )
			{
				masses[i] /= (double)masses.size();
			}

			dbody->CreateGeometries(dspace_id_, geoms, masses);
		}

		/// Set Global Body Position and Rotation.
		::vector body_position;
		::quater body_rotation;
		
		// If the body part is the pelvis or unlinked to any joint.
		if ( u_link->parent_joint_name_.empty() )
		{
			body_position = ::vector(0, 0, 0);
			body_rotation = ::quater(1, 0, 0, 0);
		}

		// If it is not pelvis.
		else
		{
			body_position = urdf.GetGlobalJointPosition( u_link->parent_joint_name_ );
			body_rotation = urdf.GetGlobalJointRotation( u_link->parent_joint_name_ );
		}

		dbody->SetPositionExt(body_position);

		dQuaternion dq;
		for ( int q=0; q<4; q++ ) dq[q] = body_rotation[q];
		dbody->SetOrietationExt(body_rotation);
	}


	/// Joints
	for ( int i=0; i<urdf.num_joints(); i++ ) 
	{
		UrdfJoint *joint = urdf.joint(i);

		int link_1_id = urdf.link_id( joint->parent_link_name_ );
		int link_2_id = urdf.link_id( joint->child_link_name_ );
		if ( link_1_id < 0 || link_2_id < 0 )
		{
			ode_joints_.push_back(0);
			continue;
		}
		if ( ode_bodies_[link_1_id] == 0 || ode_bodies_[link_2_id] == 0 )
		{
			ode_joints_.push_back(0);
			continue;
		}

		if ( strcmp(joint->type_.c_str(), "revolute") == 0 )
		{
			::vector joint_position = urdf.GetGlobalJointPosition(i);

			::vector joint_axis;
			if ( urdf.GetParentJointOfJoint(i) != 0 )
			{
				joint_axis = rotate( urdf.GetGlobalJointRotation(urdf.GetParentJointOfJoint(i)), joint->axis_ );
			}
			
			joint_axis = joint_axis.normalize();
			
			dHingeJoint *hj = AddNewHingeJoint()->d_hinge();
			dJointAttach (hj->id(), ode_bodies_[link_1_id]->id(), ode_bodies_[link_2_id]->id());
			dJointSetHingeAnchor(hj->id(), joint_position[0], joint_position[1], joint_position[2]);
			dJointSetHingeAxis(hj->id(), -1*joint_axis[0], -1*joint_axis[1], -1*joint_axis[2]);
			dJointSetHingeParam(hj->id(), dParamLoStop, joint->limit_lower_);// - atlas_angle_offsets_[i]);
			dJointSetHingeParam(hj->id(), dParamHiStop, joint->limit_upper_);// - atlas_angle_offsets_[i]);
			//dJointSetHingeParam(djoint_id, dParamFMax, 300);
			//dJointSetHingeParam(djoint_id, dParamVel, 10);

			/*
			dJointID dmotor_joint_id = dJointCreateAMotor(dworld_id_, 0);
			dJointAttach (dmotor_joint_id, ode_bodies_[link_1_id]->id(), ode_bodies_[link_2_id]->id());
			dJointSetAMotorNumAxes(dmotor_joint_id, 1);
			dJointSetAMotorAxis(dmotor_joint_id, 0, 1, -1*joint_axis[0], -1*joint_axis[1], -1*joint_axis[2]);
			dJointSetAMotorParam(dmotor_joint_id, dParamLoStop, joint->limit_lower_);
			dJointSetAMotorParam(dmotor_joint_id, dParamHiStop, joint->limit_upper_);
			*/
		}
		else if (strcmp(joint->type_.c_str(), "fixed") == 0 )
		{
			std::cout << "fixed " << joint->name_ << std::endl;
			dFixedJoint *fj = AddNewFixedJoint()->d_fixed();
			dJointAttach (fj->id(), ode_bodies_[link_1_id]->id(), ode_bodies_[link_2_id]->id());
		}
		else 
		{
			ode_joints_.push_back(0);

			delete ode_bodies_[link_2_id];
			ode_bodies_[link_2_id] = 0;

			/*
			dJointID djoint_id = dJointCreateFixed(dworld_id, 0);
			djoint_ids_.push_back(djoint_id);
			dmotor_joint_ids_.push_back(0);

			dJointAttach (djoint_id, dbody_ids_[link_1_id], dbody_ids_[link_2_id]);
			*/
		}

	}

	SetCurrentStateAsInit();
}

#endif
