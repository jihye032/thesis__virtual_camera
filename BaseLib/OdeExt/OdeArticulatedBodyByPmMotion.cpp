#ifdef ODE_EXT

#include "BaseLib/OdeExt/OdeArticulatedBody.h"
#include "BaseLib/OdeExt/OdeGL.h"
#include "BaseLib/Motion/ml.h"
#include "BaseLib/PMU/PmBoundingVolume.h"

void
OdeArticulatedBody::Create(PmHuman *body)
{
	ClearBodies();

	// Create Primitive Shapes for each body part.
	mg::PmBoundingVolume pm_bv;
	pm_bv.body(body);
	pm_bv.CreateDefaultBoundingVolumes(0.7);


	PmPosture t_pose;
	t_pose.setBody(body);
	for ( int i=0; i<PM_HUMAN_NUM_LINKS; i++ )
	{
		t_pose.setRotation(i, quater(1, 0, 0, 0));
	}
	pm_bv.SetPosture(t_pose);

	double pelvis_height = 0.0;
	{
		double lowest_bv_y = pm_bv.bounding_volume(PmHuman::PELVIS)->global_translation().y();

		for ( int i=0; i<pm_bv.num_bounding_volumes(); i++ )
		{
			if ( pm_bv.bounding_volume(i) )
			{
				if ( lowest_bv_y > pm_bv.bounding_volume(i)->global_translation().y() )
					lowest_bv_y = pm_bv.bounding_volume(i)->global_translation().y();
			}
		}

		pelvis_height = t_pose.getGlobalTranslation(PmHuman::PELVIS).y() - lowest_bv_y;
	}
	t_pose.setTranslation(::vector(0, pelvis_height, 0));

	Create(&t_pose);
}

void
OdeArticulatedBody::Create(PmPosture *pose)
{
	ClearBodies();

	// Create Primitive Shapes for each body part.
	mg::PmBoundingVolume pm_bv;
	pm_bv.body(pose->getBody());
	pm_bv.CreateDefaultBoundingVolumes(0.7);


	pm_bv.SetPosture(*pose);


	// Make sure that cylinders and capsules are laid on z-axis for the compatibility with ODE geometry.
	for ( int i=0; i<pm_bv.num_bounding_volumes(); i++ )
	{
		if ( pm_bv.bounding_volume(i) == 0 ) continue;
		
		else if ( pm_bv.bounding_volume(i)->type() == mg::PrimitiveShape::CAPSULE )
		{
			mg::PrimitiveCapsule *cap = (mg::PrimitiveCapsule *)pm_bv.bounding_volume(i);

			if ( cap->direction() == 1 )
			{
				cap->local_rotation( cap->local_rotation()*exp(x_axis*M_PI/4) );
				cap->direction(2);
			}
			else if ( cap->direction() == 0 )
			{
				cap->local_rotation( cap->local_rotation()*exp(y_axis*M_PI/4) );
				cap->direction(2);
			}
		}

		else if ( pm_bv.bounding_volume(i)->type() == mg::PrimitiveShape::CYLINDER )
		{
			mg::PrimitiveCylinder *cyl = (mg::PrimitiveCylinder *)pm_bv.bounding_volume(i);

			if ( cyl->direction() == 1 )
			{
				cyl->local_rotation( cyl->local_rotation()*exp(x_axis*M_PI/4) );
				cyl->direction(2);
			}
			else if ( cyl->direction() == 0 )
			{
				cyl->local_rotation( cyl->local_rotation()*exp(y_axis*M_PI/4) );
				cyl->direction(2);
			}
		}
	}

	for ( int i=0; i<pm_bv.num_bounding_volumes(); i++ )
	{
		if ( pm_bv.bounding_volume(i) == 0 ) continue;
		OdeBodyExt *body = AddNewBody();
		body->CreateGeometry(dspace_id_, pm_bv.bounding_volume(i), 1);
	}

	SetCurrentStateAsInit();
}




#endif