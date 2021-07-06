
#include "BaseLib/GL4U/GL_ResourceManager.h"
#include "BaseLib/GL4U/GL_RenderableObj.h"
#include "BaseLib/Motion/ml.h"

namespace mg
{
GL_VBOGroup* CreateMlBodyVBO(const ml::Body *p, double size)
{
	mg::Mesh *body_mesh = new mg::Mesh;
	{
		// Bone meshes
		{
			for ( unsigned int i=1; i<p->num_joint(); i++ )
			{
				//if ( MaskBit(i) & p->getMask() )
				{
					int parent_joint_id = p->parent(i);
					cml::vector3d p1 = p->GetGlobalTranslation( parent_joint_id );
					cml::vector3d p2 = p->GetGlobalTranslation( i );

					mg::Mesh mesh;
					mesh.CreateCylinder(p1, p2, size*2);
						
					mesh.UseBoneWeight();
					mesh.SetBoneIdAndWeightForAllVertices(0, parent_joint_id, 1.0);

					body_mesh->Merge(mesh);

				}
			}


			//// Hands
			//if ( p->getMask() & MaskBit(PmHuman::LEFT_PALM) )
			//{
			//	::vector vec = p->getJointPosition(PmHuman::LEFT_PALM) * 0.5;
			//	::vector p1 = p->getGlobalJointTransf(PmHuman::LEFT_PALM).getTranslation();
			//	::vector p2 = p1+rotate(p->getGlobalJointTransf(PmHuman::LEFT_PALM).getRotation(), vec);


			//	mg::Mesh mesh;
			//	mesh.CreateCylinder(p1, p2, size);
			//			
			//	mesh.UseBoneWeight();
			//	mesh.SetBoneIdAndWeightForAllVertices(0, PmHuman::LEFT_PALM, 1.0);

			//	body_mesh->Merge(mesh);

			//}

			//if ( p->getMask() & MaskBit(PmHuman::RIGHT_PALM) )
			//{
			//	::vector vec = p->getJointPosition(PmHuman::RIGHT_PALM) * 0.5;
			//	::vector p1 = p->getGlobalJointTransf(PmHuman::RIGHT_PALM).getTranslation();
			//	::vector p2 = p1+rotate(p->getGlobalJointTransf(PmHuman::RIGHT_PALM).getRotation(), vec);


			//	mg::Mesh mesh;
			//	mesh.CreateCylinder(p1, p2, size);
			//			
			//	mesh.UseBoneWeight();
			//	mesh.SetBoneIdAndWeightForAllVertices(0, PmHuman::RIGHT_PALM, 1.0);

			//	body_mesh->Merge(mesh);
			//}
		}
	}


	mg::GL_VBOGroup * body_vbo = mg::GL_ResourceManager::singleton()->CreateVBOGroup();// new mg::GL_VBOGroup;
	body_vbo->SetByMesh(*body_mesh);
	delete body_mesh;

	//// Bone Setting
	//renderer_->CreateAndAddRenderableObj("body0", body_vbo_);
	//for ( int i=0; i<PM_HUMAN_NUM_LINKS; i++ )
	//{
	//	if ( MaskBit(i) & motion_->getBody()->getMask() )
	//	{
	//		renderer_->GetRenderableObj("body0")->SetBoneOffsetMatrix(i, ToCml(motion_->getBody()->getGlobalJointTransf(i)));
	//	}
	//}

	return body_vbo;
}
//
//GL_RenderableObj* CreatePmHumanRObj(::PmHuman *p, double size)
//{
//	mg::Mesh *body_mesh = new mg::Mesh;
//	{
//		// Bone meshes
//		{
//			for (int i = 1; i<PM_HUMAN_NUM_LINKS; i++)
//			{
//				if (MaskBit(i) & p->getMask())
//				{
//					int parent_joint_id = p->getParent(i);
//					::vector p1 = p->getGlobalJointTransf(parent_joint_id).getTranslation();
//					::vector p2 = p->getGlobalJointTransf(i).getTranslation();
//
//					mg::Mesh mesh;
//					mesh.CreateCylinder(p1, p2, size);
//
//					mesh.UseBoneWeight();
//					mesh.SetBoneIdAndWeightForAllVertices(0, parent_joint_id, 1.0);
//
//					body_mesh->Merge(mesh);
//
//				}
//			}
//
//
//			// Hands
//			if (p->getMask() & MaskBit(PmHuman::LEFT_PALM))
//			{
//				::vector vec = p->getJointPosition(PmHuman::LEFT_PALM) * 0.5;
//				::vector p1 = p->getGlobalJointTransf(PmHuman::LEFT_PALM).getTranslation();
//				::vector p2 = p1 + rotate(p->getGlobalJointTransf(PmHuman::LEFT_PALM).getRotation(), vec);
//
//
//				mg::Mesh mesh;
//				mesh.CreateCylinder(p1, p2, size);
//
//				mesh.UseBoneWeight();
//				mesh.SetBoneIdAndWeightForAllVertices(0, PmHuman::LEFT_PALM, 1.0);
//
//				body_mesh->Merge(mesh);
//
//			}
//
//			if (p->getMask() & MaskBit(PmHuman::RIGHT_PALM))
//			{
//				::vector vec = p->getJointPosition(PmHuman::RIGHT_PALM) * 0.5;
//				::vector p1 = p->getGlobalJointTransf(PmHuman::RIGHT_PALM).getTranslation();
//				::vector p2 = p1 + rotate(p->getGlobalJointTransf(PmHuman::RIGHT_PALM).getRotation(), vec);
//
//
//				mg::Mesh mesh;
//				mesh.CreateCylinder(p1, p2, size);
//
//				mesh.UseBoneWeight();
//				mesh.SetBoneIdAndWeightForAllVertices(0, PmHuman::RIGHT_PALM, 1.0);
//
//				body_mesh->Merge(mesh);
//			}
//		}
//	}
//
//
//	mg::GL_VBOGroup * body_vbo = mg::GL_ResourceManager::singleton()->CreateVBOGroup();// new mg::GL_VBOGroup;
//	body_vbo->SetByMesh(*body_mesh);
//
//	mg::GL_RenderableObj *r_obj = mg::GL_ResourceManager::singleton()->CreateRenderableObj("body0", body_vbo, nullptr);// new mg::GL_VBOGroup;
//	r_obj->flag_skinning(true);
//
//	//// Bone Setting
//	//for ( int i=0; i<PM_HUMAN_NUM_LINKS; i++ )
//	//{
//	//	if ( MaskBit(i) & motion_->getBody()->getMask() )
//	//	{
//	//		renderer_->GetRenderableObj("body0")->SetBoneOffsetMatrix(i, ToCml(motion_->getBody()->getGlobalJointTransf(i)));
//	//	}
//	//}
//
//	return r_obj;
//}
};