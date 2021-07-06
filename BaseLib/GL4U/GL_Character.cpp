
#pragma once

#include "GL/glew.h"
#include "BaseLib/GL4U/GL_Character.h"
#include "BaseLib/GL4U/GL_ResourceManager.h"
#include "BaseLib/GL4U/GL_Renderer.h"
#include <algorithm>
#include <stack>

namespace mg
{

GL_Character::GL_Character()
{
}

void
GL_Character::LoadModelbyAssimp(std::string filename)
{
	GL_ResourceManager::singleton()->ImportByAssimp(&character_model_root_, filename, "Remy");

	// Read all bone infos
	std::vector<GL_RenderableObj*> r_objs;
	character_model_root_.GetAllRenderableObjsInDFS(r_objs);
	for ( auto &d : r_objs )
	{
		std::cout << d->name() << std::endl;

		for ( int i :  d->using_bone_matrix_id_set() )
		{
			std::cout << "  " << i << ": " << d->GetBoneName(i) << std::endl;

			bone_name_to_id_map_[d->GetBoneName(i)] = i;
			bone_id_to_name_map_[i] = d->GetBoneName(i);
			bone_id_to_offset_map_[i] = d->GetBoneOffsetMatrix(i);
		}
	}

	std::vector<GL_SceneNode*> s_nodes;
	character_model_root_.GetNodeHierarchyInDFS(s_nodes);
	for ( auto & node : s_nodes )
	{
		auto bone_name_id = bone_name_to_id_map_.find(node->name());
		if ( bone_name_id != bone_name_to_id_map_.end() )
		{
			bone_name_to_node_map_[bone_name_id->first] = node;
		}
	}
}

void 
GL_Character::SetJointTag(std::string bone_name, ml::JointTag t)
{
	auto bone_name_id = bone_name_to_id_map_.find(bone_name);
	if ( bone_name_id == bone_name_to_id_map_.end() ) 
	{
		std::cerr << bone_name << ", bone havs not been found!!!!!!!!!!!! [SetJointTag()]" << std::endl;
	}

	joint_tag_to_bone_id_map_[t] = bone_name_to_id_map_[ bone_name ];

	bone_id_to_joint_tag_map_[ bone_name_to_id_map_[ bone_name ] ]= t;
}

ml::JointTag 
GL_Character::GetJointTagByBoneName(std::string bone_name) const
{
	auto bone_name_id = bone_name_to_id_map_.find( bone_name );
	if ( bone_name_id == bone_name_to_id_map_.end() ) return ml::UNKNOWN;

	auto bone_id_joint_tag = bone_id_to_joint_tag_map_.find( bone_name_id->second );

	if ( bone_id_joint_tag == bone_id_to_joint_tag_map_.end() ) return ml::UNKNOWN;
	
	return bone_id_joint_tag->second;
}

std::string 
GL_Character::GetBoneName(ml::JointTag t) const
{
	auto d = joint_tag_to_bone_id_map_.find(t);
	
	if ( d == joint_tag_to_bone_id_map_.end() )
		return std::string();
	else 
		return bone_id_to_name_map_.find(d->second)->second;
}


void 
GL_Character::DiscardUntaggedBones()
{

	for ( auto bone_name_id : bone_name_to_id_map_ )
	{
		// for untagged each untagged bone node.
		if ( GetJointTagByBoneName(bone_name_id.first) == ml::UNKNOWN ) 
		{
			// untagged bone node.
			const GL_SceneNode *untagged_bone_node = bone_name_to_node_map_[bone_name_id.first];
			if ( untagged_bone_node == nullptr ) continue;

			// find the nearest tagged parent bone.
			const GL_SceneNode *parent_bone_node = GetNearestTaggedParentBoneNode(untagged_bone_node);

			// If there is no tagged parent.
			if ( parent_bone_node == nullptr ) continue;

			// Replace the untagged bone to the parent bone in all meshes
			ReplaceBone(bone_name_id.first, parent_bone_node->name());
			
		}

	}
}

void 
GL_Character::BuildSkeleton()
{
	skeleton_.Clear();
	
	// Pelvis (root)
	// root joint must be at the first place of skeleton_.m_joints
	{
		ml::Joint root_joint;
		root_joint.offset.set(0, 0, 0);
		root_joint.parent = -1;
		skeleton_.editable_joints().push_back(root_joint);
		skeleton_.editable_joint_name_map()[ GetBoneName(ml::PELVIS) ] = 0;
		skeleton_.editable_joint_tag_map()[ ml::PELVIS ] = 0;
	}

	// Set Joints (index, tag, name)
	for ( auto tag_boneId : joint_tag_to_bone_id_map_ )
	{
		if ( tag_boneId.first == ml::PELVIS  ) continue;

		int joint_id = (int)skeleton_.editable_joints().size();

		skeleton_.editable_joint_name_map()[ GetBoneName(tag_boneId.first) ] = joint_id;
		skeleton_.editable_joint_tag_map()[ tag_boneId.first ] = joint_id;
		skeleton_.editable_joints().push_back(ml::Joint());
	}

	// Set Parents and Offsets
	for ( int j=1; j<(int)skeleton_.num_joint(); j++ )
	{
		ml::Joint &cur_joint = skeleton_.editable_joints()[j];

		ml::JointTag cur_tag = skeleton_.joint_tag(j);
		ml::JointTag parent_tag = GetNearestTaggedParentTag(cur_tag);

		cur_joint.parent = skeleton_.joint_index(parent_tag);

		cml::matrix44d cur_offset = bone_id_to_offset_map_[ joint_tag_to_bone_id_map_[cur_tag] ];
		cml::matrix44d parent_offset = bone_id_to_offset_map_[ joint_tag_to_bone_id_map_[parent_tag] ];

		cur_joint.offset = cml::trans(cur_offset) - cml::trans(parent_offset);
	}

	posture_.body(&skeleton_);
	posture_.trans({0., 0., 0.});
	for ( int j=0; j<(int)posture_.num_joint(); j++ )
	{
		posture_.rotate(j, cml::identity_3x3());
	}
}

void 
GL_Character::UpdateCharacterPose()
{
	std::vector<GL_RenderableObj*> r_objs;
	character_model_root_.GetAllRenderableObjsInDFS(r_objs);

	for ( unsigned int j=0; j<skeleton_.num_joint(); j++ )
	{
		ml::JointTag j_tag = skeleton_.joint_tag(j);
		int bone_id = joint_tag_to_bone_id_map_[j_tag];
		cml::matrix44d m = posture_.GetGlobalTransf(j);

		for ( auto &r : r_objs )
		{
			if ( r->using_bone_matrix_id_set().find(bone_id)
				!= r->using_bone_matrix_id_set().end() )
			{
				r->SetBoneMatrix(bone_id, m);
			}
		}
	}

	
	
}

void 
GL_Character::SetBoneMatrix(int bone_id, cml::matrix44d m)
{
	std::vector<GL_RenderableObj*> r_objs;
	character_model_root_.GetAllRenderableObjsInDFS(r_objs);
	for ( auto &r : r_objs )
	{
		if ( r->using_bone_matrix_id_set().find(bone_id) 
			!= r->using_bone_matrix_id_set().end() )
		{
			r->SetBoneMatrix(bone_id, m);
		}
	}
}

double GL_Character::CalculScaleTo(const ml::Body * body) const
{
	// Scale estimation
	double scale = 1.0;

	

	scale = ( cml::length(body->joint(ml::R_KNEE).offset) + cml::length(body->joint(ml::R_ANKLE).offset) )
			/ 
			( cml::length( character_model_root_.transf()*skeleton_.joint(ml::R_KNEE).offset )
			+ cml::length(character_model_root_.transf()*skeleton_.joint(ml::R_ANKLE).offset) );
	
	return scale;
}

void 
GL_Character::Retarget(const ml::Posture & in, bool limb_ik, bool head_ik)
{
	// Scale estimation
	double scale = 1.0;
	{
		scale = ( cml::length(in.body()->joint(ml::R_KNEE).offset) 
					+ cml::length(in.body()->joint(ml::R_ANKLE).offset) )
				/ 
				( cml::length( skeleton_.joint(ml::R_KNEE).offset )
					+ cml::length(skeleton_.joint(ml::R_ANKLE).offset) );
	}

	posture_.trans(in.trans()/scale);

	for ( unsigned int i=0; i<in.num_joint(); i++ )
	{
		ml::JointTag j_tag = in.body()->joint_tag(i);

		if ( j_tag != ml::UNKNOWN && skeleton_.HasTag( j_tag ) )
		{
			posture_.rotate( skeleton_.joint_index(j_tag), in.rotate(i));
		}
	}

	

	if ( head_ik ) {

		/*
		ml::Constraint c;
		for ( ml::JointTag d : ml::GetEssentialJointTagSet() )
		{
			c.Push( posture_.body()->joint_index(d    ), cml::make_transf(in.GetGlobalRoation(d), in.GetGlobalTranslation(d    )/scale) );
		}
		posture_.IkFullBody(c); */

		posture_.IkLimb(ml::HEAD, ml::CHEST, ml::SPINE, in.GetGlobalTranslation(ml::HEAD)/scale);
		posture_.SetGlobalRotation(ml::HEAD, in.GetGlobalRoation(ml::HEAD));
	}

	if ( limb_ik ) {
		posture_.IkLimb(ml::L_WRIST, in.GetGlobalTranslation(ml::L_WRIST)/scale);
		posture_.IkLimb(ml::R_WRIST, in.GetGlobalTranslation(ml::R_WRIST)/scale);
		posture_.IkLimb(ml::L_ANKLE, in.GetGlobalTranslation(ml::L_ANKLE)/scale);
		posture_.IkLimb(ml::R_ANKLE, in.GetGlobalTranslation(ml::R_ANKLE)/scale);

		posture_.SetGlobalRotation(ml::L_WRIST, in.GetGlobalRoation(ml::L_WRIST));
		posture_.SetGlobalRotation(ml::R_WRIST, in.GetGlobalRoation(ml::R_WRIST));
		posture_.SetGlobalRotation(ml::L_ANKLE, in.GetGlobalRoation(ml::L_ANKLE));
		posture_.SetGlobalRotation(ml::R_ANKLE, in.GetGlobalRoation(ml::R_ANKLE));
	}

	UpdateCharacterPose();
}

void
GL_Character::Draw(GL_Renderer * r)
{
	r->Draw(&character_model_root_);
}



void
GL_Character::ReplaceBone(std::string target_bone_name, std::string src_bone_name)
{
	std::vector<mg::GL_RenderableObj*> r_objs;
	character_model_root_.GetAllRenderableObjsInDFS(r_objs);

	int target_bone_id = bone_name_to_id_map_[target_bone_name];
	int src_bone_id = bone_name_to_id_map_[src_bone_name];

	for ( auto &r : r_objs )
	{
		if ( target_bone_id==src_bone_id ) continue;

		r->ReplaceBone(target_bone_id, 
			src_bone_id, 
			src_bone_name, 
			bone_id_to_offset_map_[src_bone_id]);
	}

}

const GL_SceneNode * 
GL_Character::GetNearestTaggedParentBoneNode(const GL_SceneNode * c) const
{

	const GL_SceneNode *parent_bone_node = c->parent();
	{
		while ( parent_bone_node != nullptr )
		{
			if  ( GetJointTagByBoneName(parent_bone_node->name()) != ml::UNKNOWN ) break;
			parent_bone_node = parent_bone_node->parent();
		}
	}

	return parent_bone_node;
}

ml::JointTag 
GL_Character::GetNearestTaggedParentTag(ml::JointTag c) const
{
	ml::JointTag res = ml::UNKNOWN;
	const GL_SceneNode* cur_bone_node = GetBoneNodeByJointTag(c);

	if ( cur_bone_node == nullptr ) return res;

	const GL_SceneNode* parent_bone_node = GetNearestTaggedParentBoneNode(cur_bone_node);

	res = GetJointTagByBoneName( parent_bone_node->name() );

	return res;
}

const GL_SceneNode * 
GL_Character::GetBoneNodeByJointTag(ml::JointTag c) const
{
	auto p = bone_name_to_node_map_.find(GetBoneName(c));
	if ( p == bone_name_to_node_map_.end() ) return nullptr;

	return p->second;
}

};





