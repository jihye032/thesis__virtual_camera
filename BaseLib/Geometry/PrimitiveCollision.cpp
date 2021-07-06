
#include "BaseLib/Geometry/PrimitiveShape.h"
#include "BaseLib/Geometry/Line.h"

namespace mg
{
	

//////////////////////////////////////////////////////////////////////
// Collision

bool CheckCollision(PrimitiveShape *a, PrimitiveShape *b, ContactInfo &out_contact_info)
{
	cml::vector3d d;
	if ( !CheckCollision(a, b, d)  ) return false;

	out_contact_info.depth_ = cml::length(d);
	out_contact_info.normal_ = cml::normalize(d);

	return true;
}

bool CheckCollision(PrimitiveShape *a, PrimitiveShape *b, cml::vector3d &out_penetration_depth)
{
	if ( a->type() == PrimitiveShape::CAPSULE && a->type() == PrimitiveShape::CAPSULE )
	{
		return CheckCollisionCapCap((PrimitiveCapsule *)a, (PrimitiveCapsule *)b, out_penetration_depth);
	}
	else if ( a->type() == PrimitiveShape::SPHERE && a->type() == PrimitiveShape::SPHERE )
	{
		return CheckCollisionSphSph((PrimitiveSphere *)a, (PrimitiveSphere *)b, out_penetration_depth);
	}

	else if ( a->type() == PrimitiveShape::CAPSULE && b->type() == PrimitiveShape::SPHERE )
	{
		return CheckCollisionCapSph((PrimitiveCapsule *)a, (PrimitiveSphere *)b, out_penetration_depth);
	}
	else if ( a->type() == PrimitiveShape::SPHERE && b->type() == PrimitiveShape::CAPSULE )
	{
		bool result = CheckCollisionCapSph((PrimitiveCapsule *)b, (PrimitiveSphere *)a, out_penetration_depth);

		out_penetration_depth *= -1;
		return result;
	}

	else if ( a->type() == PrimitiveShape::CAPSULE && b->type() == PrimitiveShape::BOX )
	{
		return CheckCollisionCapBox((PrimitiveCapsule *)a, (PrimitiveBox *)b, out_penetration_depth);
	}
	else if ( a->type() == PrimitiveShape::BOX && b->type() == PrimitiveShape::CAPSULE )
	{
		bool result = CheckCollisionCapBox((PrimitiveCapsule *)b, (PrimitiveBox *)a, out_penetration_depth);

		out_penetration_depth *= -1;
		return result;
	}

	else if ( a->type() == PrimitiveShape::SPHERE && b->type() == PrimitiveShape::BOX )
	{
		return CheckCollisionSphBox((PrimitiveSphere *)a, (PrimitiveBox *)b, out_penetration_depth);
	}
	else if ( a->type() == PrimitiveShape::BOX && b->type() == PrimitiveShape::SPHERE )
	{
		bool result = CheckCollisionSphBox((PrimitiveSphere *)b, (PrimitiveBox *)a, out_penetration_depth);

		out_penetration_depth *= -1;
		return result;
	}

	return false;
}

bool CheckCollisionCapCap(PrimitiveCapsule *A_cap, PrimitiveCapsule *B_cap, cml::vector3d &out_penetration_depth)
{
	mg::LineSegment A_line;
	{
		cml::vector3d p0, p1;
		A_cap->GetGlobalCylinderEndPoints(p0, p1);
		A_line.points((p0), (p1));
	}
	mg::LineSegment B_line;
	{
		cml::vector3d p0, p1;
		B_cap->GetGlobalCylinderEndPoints(p0, p1);
		B_line.points((p0), (p1));
	}

	mg::LineSegment bridge = A_line.BridgeToLineSeg(B_line);

	if ( bridge.length() < A_cap->radius()+B_cap->radius() )
	{
		double depth = A_cap->radius()+B_cap->radius() - bridge.length();
		out_penetration_depth = cml::normalize(bridge.point_A()-bridge.point_B()) * depth;
		return true;
	}

	return false;
}

bool CheckCollisionSphSph(PrimitiveSphere *A_sph, PrimitiveSphere *B_sph, cml::vector3d &out_penetration_depth)
{
	cml::vector3d p_a = A_sph->global_translation();
	cml::vector3d p_b = B_sph->global_translation();

	if ( cml::length(p_a-p_b) < A_sph->radius() + B_sph->radius() )
	{
		double depth = A_sph->radius()+B_sph->radius() - cml::length(p_a-p_b);
		out_penetration_depth = cml::normalize(p_a-p_b) * depth;

		return true;
	}

	return false;
}

static bool CheckCollisionCapSph(PrimitiveCapsule *A_cap, PrimitiveSphere *B_sph, cml::vector3d &out_penetration_depth)
{
	mg::LineSegment A_line;
	{
		cml::vector3d p0, p1;
		A_cap->GetGlobalCylinderEndPoints(p0, p1);
		A_line.points((p0), (p1));
	}

	LineSegment bridge = A_line.BridgeToPoint((B_sph->global_translation()));
	if ( bridge.length() < A_cap->radius() + B_sph->radius() )
	{
		double depth = A_cap->radius()+B_sph->radius() - bridge.length();
		out_penetration_depth = cml::normalize(bridge.point_A()-bridge.point_B()) * depth;

		return true;
	}

	return false;
}


bool CheckCollisionCapBox(PrimitiveCapsule *A_cap, PrimitiveBox *B_box, cml::vector3d &out_penetration_depth)
{
	PrimitiveSphere bounding_sph = B_box->CalculBoundingSphere();
	bool result = CheckCollisionCapSph(A_cap, &bounding_sph, out_penetration_depth);
	if ( !result ) return result;

	cml::vector3d p0, p1;
	mg::LineSegment A_line;
	{
		A_cap->GetGlobalCylinderEndPoints(p0, p1);
		A_line.points((p0), (p1));

		A_line.Translate(-1*B_box->global_translation());
		A_line.Rotate(B_box->global_rotation().inverse());
		A_line.Scale(cml::vector3d(1.0/(A_cap->radius()+B_box->width()), 
							1.0/(A_cap->radius()+B_box->height()), 
							1.0/(A_cap->radius()+B_box->depth()))
							);

		p0 = ( A_line.point_A() );
		p1 = ( A_line.point_B() );
	}
	

	cml::vector3d *min_x = &p0, *max_x = &p1;
	if ( p0[0] > p1[0] )
	{
		min_x = &p1;
		max_x = &p0;
	}

	if ( (*min_x)[0] > 0.5 || (*max_x)[0] < -0.5 )
	{
		return false;
	}
	else
	{
		cml::vector3d dir_x = cml::normalize( (*max_x)-(*min_x) );

		if ( (*min_x)[0] < -0.5 )  
		{
			(*min_x) += dir_x*(-0.5-(*min_x)[0])/dir_x[0];
		}
		if ( (*max_x)[0] > 0.5 )  
		{
			(*max_x) += dir_x*(0.5-(*max_x)[0])/dir_x[0];
		}


		cml::vector3d *min_y = &p0, *max_y = &p1;
		if ( p0[1] > p1[1] )
		{
			min_y = &p1;
			max_y = &p0;
		}

		if ( (*min_y)[1] > 0.5 || (*max_y)[1] < -0.5 )
		{
			return false;
		}
		else
		{
			cml::vector3d dir_y = cml::normalize( (*max_y)-(*min_y) );
			
			if ( (*min_y)[1] < -0.5 )  
			{
				(*min_y) += dir_y*(-0.5-(*min_y)[1])/dir_y[1];
			}
			if ( (*max_y)[1] > 0.5 )  
			{
				(*max_y) += dir_y*(0.5-(*max_y)[1])/dir_y[1];
			}

			cml::vector3d *min_z = &p0, *max_z = &p1;
			if ( p0[2] > p1[2] )
			{
				min_z = &p1;
				max_z = &p0;
			}

			if ( (*min_z)[2] > 0.5 || (*max_z)[2] < -0.5 )
			{
				return false;
			}
			else
			{
				return true;
			}
		}
	}
	
	

	return false;
}


bool CheckCollisionSphBox(PrimitiveSphere *A_sph, PrimitiveBox *B_box, cml::vector3d &out_penetration_depth)
{
	PrimitiveSphere bounding_sph = B_box->CalculBoundingSphere();
	bool result = CheckCollisionSphSph(A_sph, &bounding_sph, out_penetration_depth);
	if ( !result ) return result;

	cml::vector3d a_sphere_center = A_sph->global_translation();


	a_sphere_center -= B_box->global_translation();
	a_sphere_center = cml::Rotate(B_box->global_rotation().inverse(), a_sphere_center);
	a_sphere_center[0] *= 1.0/(A_sph->radius()+B_box->width());
	a_sphere_center[1] *= 1.0/(A_sph->radius()+B_box->height());
	a_sphere_center[2] *= 1.0/(A_sph->radius()+B_box->depth());
	
	if ( 0.5 > a_sphere_center[0] && a_sphere_center[0] > -0.5
		&& 0.5 > a_sphere_center[1] && a_sphere_center[1] > -0.5
		&& 0.5 > a_sphere_center[2] && a_sphere_center[2] > -0.5 )
	{
		out_penetration_depth = cml::vector3d(0.5 - a_sphere_center[0], 0, 0);

		if ( out_penetration_depth.length() > a_sphere_center[0] + 0.5 )
			out_penetration_depth = cml::vector3d(a_sphere_center[0] + 0.5, 0, 0);

		if ( out_penetration_depth.length() > 0.5 - a_sphere_center[1] )
			out_penetration_depth = cml::vector3d(0, 0.5 - a_sphere_center[1], 0);

		if ( out_penetration_depth.length() > a_sphere_center[1] + 0.5 )
			out_penetration_depth = cml::vector3d(0, a_sphere_center[1] + 0.5, 0);

		if ( out_penetration_depth.length() > 0.5 - a_sphere_center[2] )
			out_penetration_depth = cml::vector3d(0, 0, 0.5 - a_sphere_center[2]);

		if ( out_penetration_depth.length() > a_sphere_center[2] + 0.5 )
			out_penetration_depth = cml::vector3d(0, 0, a_sphere_center[2] + 0.5);


		out_penetration_depth[0] *= (A_sph->radius()+B_box->width());
		out_penetration_depth[1] *= (A_sph->radius()+B_box->height());
		out_penetration_depth[2] *= (A_sph->radius()+B_box->depth());
		out_penetration_depth = cml::Rotate(B_box->global_rotation(), out_penetration_depth);
		out_penetration_depth += B_box->global_translation();

		return true;
	}

	return false;
}


};