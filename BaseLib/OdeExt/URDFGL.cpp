#ifdef ODE_EXT
#include "BaseLib/OdeExt/URDFGL.h"
#include "BaseLib/Geometry/GeometryGL.h"
#include "BaseLib/GLUU/gluu.h"
#include "BaseLib/Geometry/GeometryGL.h"
#include <sstream>

void DrawUrdfBody(const Urdf &atlas, bool colored)
{
	for ( int i=0; i<atlas.num_links(); i++ )
	{
		UrdfLink *cur_link = atlas.link(i);
		if ( cur_link->bounding_volume_ == 0 ) continue;

		mg::mgluPush();

		int parent_joint_id = atlas.joint_id( cur_link->parent_joint_name_ );
		if ( parent_joint_id != -1 )
		{
			mg::mgluTranslateV(atlas.GetGlobalJointPosition(parent_joint_id));
			mg::mgluRotateQ(atlas.GetGlobalJointRotation(parent_joint_id));
		}

		if ( colored )
		{
			mg::mgluSimpleMaterial(mg::mgluGetRecentlyUsedColorIndex()+1);
		}
		mg::DrawSolidObject( *(cur_link->mesh_model_) );

		mg::mgluPop();
	}
}


void DrawUrdfDefaultBoundVolumes(const Urdf &atlas, bool colored)
{
	for ( int i=0; i<atlas.num_links(); i++ )
	{
		UrdfLink *cur_link = atlas.link(i);
		if ( cur_link->bounding_volume_ == 0 ) continue;



		mg::mgluPush();

		int parent_joint_id = atlas.joint_id( cur_link->parent_joint_name_ );
		if ( parent_joint_id != -1 )
		{
			mg::mgluTranslateV(atlas.GetGlobalJointPosition(parent_joint_id));
			mg::mgluRotateQ(atlas.GetGlobalJointRotation(parent_joint_id));
		}

		if ( colored )
		{
			mg::mgluSimpleMaterial(mg::mgluGetRecentlyUsedColorIndex()+1);
		}

		mg::DrawPrimitiveShape( *cur_link->bounding_volume_ );

		mg::mgluPop();
	}
}

void DrawUrdfOtherBoundVolumes(const Urdf &atlas, bool colored)
{
	for ( int i=0; i<atlas.num_links(); i++ )
	{
		UrdfLink *cur_link = atlas.link(i);
		if ( cur_link->bounding_volume_ == 0 ) continue;



		mg::mgluPush();

		int parent_joint_id = atlas.joint_id( cur_link->parent_joint_name_ );
		if ( parent_joint_id != -1 )
		{
			mg::mgluTranslateV(atlas.GetGlobalJointPosition(parent_joint_id));
			mg::mgluRotateQ(atlas.GetGlobalJointRotation(parent_joint_id));
		}


		if ( colored )
		{
			mg::mgluSimpleMaterial(mg::mgluGetRecentlyUsedColorIndex()+1);
		}

		for ( unsigned int j=0; j<cur_link->other_bounding_volumes_.size(); j++ )
		{
			mg::DrawPrimitiveShape( *cur_link->other_bounding_volumes_[j] );
		}


		mg::mgluPop();
	}
}










#endif
