
#include "BaseLib/PMU/MlBoundingVolume.h"
#include <algorithm>

namespace mg
{
	static cml::vector3d  x_axis(1., 0., 0.);
	static cml::vector3d  y_axis(0., 1., 0.);
	static cml::vector3d  z_axis(0., 0., 1.);
MlBoundingVolume::MlBoundingVolume()
{
	body_= 0;
}

MlBoundingVolume::~MlBoundingVolume()
{
	Clear();
}

void
MlBoundingVolume::Clear()
{
	for ( unsigned int i=0; i<bounding_volumes_.size(); i++ )
	{
		if ( bounding_volumes_[i] != 0 )
			delete bounding_volumes_[i];
	}

	bounding_volumes_.clear();

	body_ = 0;
}

void
MlBoundingVolume::body(const ml::Body *body)
{
	Clear();

	body_ = body;
}

void
MlBoundingVolume::bounding_volume(int joint_id, const PrimitiveShape &s)
{
	while ( (int)bounding_volumes_.size() <= joint_id )
	{
		bounding_volumes_.push_back(0);
	}

	if ( bounding_volumes_[joint_id] != 0 ) delete bounding_volumes_[joint_id];

	bounding_volumes_[joint_id] = s.CreateClone();
}

void
MlBoundingVolume::SetPosture(const ml::Posture &pose)
{
	if ( bounding_volumes_.empty() ) return;

	for ( unsigned int i=0; i<bounding_volumes_.size(); i++ )
	{
		{
			bounding_volumes_[i]->rotation( cml::QuaternionMatrix( pose.GetGlobalRoation(i) ) );
			bounding_volumes_[i]->translation( pose.GetGlobalTranslation(i) );
		}
	}
}

mg::PrimitiveShape* 
MlBoundingVolume::CreateBonePrimitive(cml::vector3d bone_v, double radius)
{
	double bone_length = cml::length(bone_v);
	cml::quaterniond bone_orient = cml::EXP( 0.5*( cml::normalize(cml::cross(y_axis,bone_v))*atan2(cml::length(cml::cross(y_axis,bone_v)), cml::dot(y_axis,bone_v)) ) );

	if ( bone_length-2*radius > 0 )
	{
		mg::PrimitiveCapsule *capsule = new mg::PrimitiveCapsule();
		capsule->radius(radius);
		capsule->cylinder_height( bone_length-2*radius );
		capsule->TranslateLocally( cml::vector3( 0, bone_length/2, 0 ) );
		capsule->RotateLocally( bone_orient );
		return capsule;
	}
	else 
	{
		mg::PrimitiveSphere *sphere = new mg::PrimitiveSphere();
		sphere->radius( bone_length/2 );
		sphere->TranslateLocally( cml::vector3( 0, bone_length/2, 0 ) );
		sphere->RotateLocally( bone_orient );
		return sphere;
	}
}

mg::PrimitiveShape* 
MlBoundingVolume::CreateBonePrimitive(cml::vector3d p0, cml::vector3d p1, double radius)
{
	mg::PrimitiveShape* s = CreateBonePrimitive(p1-p0, radius);
	s->TranslateLocally(p0);
	return s;
}

mg::PrimitiveShape* 
MlBoundingVolume::CreateBonePrimitive(std::vector< cml::vector3> &sampling_points)
{
	cml::vector3d min_corner;
	cml::vector3d max_corner;
	min_corner = max_corner = sampling_points.front();

	

	for ( unsigned int i=1; i<sampling_points.size(); i++ )
	{
		for ( int j=0; j<3; j++ )
		{
			min_corner[j] = std::min(min_corner[j], sampling_points[i][j]);
			max_corner[j] = std::max(max_corner[j], sampling_points[i][j]);
		}
	}

	cml::vector3d center = 0.5*(max_corner+min_corner);
	cml::vector3d diff = max_corner-min_corner;
			
	int longest_axis_id = 0;
	int longer_axis_id = 1;
	int shortest_axis_id = 2;

	if ( diff[0] > diff[1] && diff[0] > diff[2] )
	{
		longest_axis_id = 0;
		if ( diff[1] > diff[2] )
		{
			longer_axis_id = 1;
			shortest_axis_id = 2;
		}
		else
		{ 
			longer_axis_id = 2;
			shortest_axis_id = 1;
		}
	}
	else if ( diff[1] > diff[0] && diff[1] > diff[2] )
	{
		longest_axis_id = 1;
		if ( diff[0] > diff[2] )
		{
			longer_axis_id = 0;
			shortest_axis_id = 2;
		}
		else
		{ 
			longer_axis_id = 2;
			shortest_axis_id = 0;
		}
	}
	else
	{
		longest_axis_id = 2;
		if ( diff[0] > diff[1] )
		{
			longer_axis_id = 0;
			shortest_axis_id = 1;
		}
		else
		{ 
			longer_axis_id = 1;
			shortest_axis_id = 0;
		}
	}

	double r = diff[longer_axis_id] / 2;
	cml::vector3d p0, p1;

	p0[longest_axis_id]  = min_corner[longest_axis_id];
	p1[longest_axis_id]  = max_corner[longest_axis_id];

	p0[longer_axis_id]   = center[longer_axis_id];
	p1[longer_axis_id]   = center[longer_axis_id];

	p0[shortest_axis_id] = center[shortest_axis_id];
	p1[shortest_axis_id] = center[shortest_axis_id];

	
	return CreateBonePrimitive(p0, p1, r);
}



void
MlBoundingVolume::CreateDefaultBoundingVolumes(double radius_scale)
{
	if ( body_ == 0 ) return;
	const ml::Body *body = body_;
	std::vector<PrimitiveShape*> &body_bounding_volumes = bounding_volumes_;
	
	// Defaulat Radiuses for Bounding Cylinder or Sphere of each bone. 
	double big_body_radius = 10 * radius_scale;
	double small_body_radius = 5 * radius_scale;
	double head_radius = big_body_radius * 1.2 * radius_scale;

	// Radiuses could be defined proportional to the length of the longest bone. 
	double longest_bone_len = 1.;
	for ( unsigned int i=1; i<body->num_joint(); i++ )
	{
		double tmp = cml::length(body->offset(i));
		if ( i==1 || tmp > longest_bone_len )
		{
			longest_bone_len = tmp;
		}
	}
	//if ( body->getMask() & MaskBit(ml::Body::LOWER_RIGHT_LEG) )
	{
		big_body_radius = radius_scale * longest_bone_len / 4.44;
		small_body_radius = radius_scale * big_body_radius * 0.5;
		head_radius = radius_scale * big_body_radius * 1.2;
	}

	// Reset
	if ( !body_bounding_volumes.empty() )
	{
		for ( unsigned int i=0; i<body_bounding_volumes.size(); i++ )
		{
			if ( body_bounding_volumes[i] != 0 ) delete body_bounding_volumes[i];
		}
	}

	body_bounding_volumes.resize(body->num_joint());
	for ( unsigned int i=0; i<body_bounding_volumes.size(); i++ )
	{
		body_bounding_volumes[i] = 0;
	}


	/// Pelvis and Chest Parts.
	// Becaue these have more than two children joints,
	// we need to treat them specially.
	//{
	//	// Collect children of the Pelvis and Chest.
	//	std::vector< cml::vector3> child_joints_of_pelvis;
	//	std::vector< cml::vector3> child_joints_of_chest;
	//	for ( int i = 1; i < body->num_joint(); i++ )
	//	{
	//		//if ( !(body->getMask() & MaskBit(i)) ) continue;

	//		int parent_id = body->parent(i);
	//		if ( parent_id == 0 )
	//		{
	//			child_joints_of_pelvis.push_back(body->getJointPosition(i));
	//		}
	//		else if ( parent_id == ml::Body::CHEST )
	//		{
	//			child_joints_of_chest.push_back(body->getJointPosition(i));
	//		}
	//	}

	//	// Pelvis Body
	//	if ( !child_joints_of_pelvis.empty() )
	//	{
	//		body_bounding_volumes[ml::Body::PELVIS] = CreateBonePrimitive(child_joints_of_pelvis);
	//	}

	//	// Chest Body
	//	if ( !child_joints_of_chest.empty() )
	//	{
	//		body_bounding_volumes[ml::Body::CHEST] = CreateBonePrimitive(child_joints_of_chest);
	//	}

	//}


	// Rest Parts.
	for ( unsigned int i = 1; i < body->num_joint(); i++ )
	{
		// if ( !(body->getMask() & MaskBit(i)) ) continue;

		int parent_id = body->parent(i);
		cml::vector3d v_from_parent = body->offset(i);
		
		/*if ( parent_id==ml::Body::LEFT_SHOULDER || parent_id==ml::Body::RIGHT_SHOULDER || parent_id==ml::Body::NECK )
		{
			body_bounding_volumes[parent_id] = CreateBonePrimitive(v_from_parent, small_body_radius);
		}
		else if ( i==ml::Body::LEFT_PALM || i==ml::Body::RIGHT_PALM )
		{
			body_bounding_volumes[parent_id] = CreateBonePrimitive(v_from_parent, big_body_radius);
			body_bounding_volumes[i] = CreateBonePrimitive(v_from_parent*(5.0/9.0), small_body_radius);
		}
		else if ( i==ml::Body::LEFT_TOE || i==ml::Body::RIGHT_TOE )
		{
			body_bounding_volumes[parent_id] = CreateBonePrimitive(v_from_parent, small_body_radius);
			body_bounding_volumes[i] = CreateBonePrimitive(v_from_parent, small_body_radius);
		}
		else if ( i==ml::Body::HEAD )
		{
			if ( parent_id != ml::Body::CHEST )
				body_bounding_volumes[parent_id] = CreateBonePrimitive(v_from_parent, small_body_radius);

			body_bounding_volumes[i] = CreateBonePrimitive(v_from_parent.normalize()*head_radius*2, head_radius);
		}
		else if ( parent_id != ml::Body::CHEST && parent_id != ml::Body::PELVIS )*/
		{
			body_bounding_volumes[parent_id] = CreateBonePrimitive(v_from_parent, big_body_radius);
		}
	}
}


};
