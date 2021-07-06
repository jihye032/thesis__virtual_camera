
#pragma once

#include "GL/glew.h"
#include "BaseLib/CmlExt/CmlExt.h"
#include "BaseLib/GL4U/GL_SceneGraph.h"
#include "BaseLib/Motion/ml.h"
#include <memory>
#include <vector>

namespace mg
{

class GL_Renderer;

class GL_Character
{
	friend class GL_ResourceManager;
public:
	GL_Character();

	//////////////////////////////////////////////////
	// Initialize

	/**
	Load model by Assimp 
	*/
	void LoadModelbyAssimp(std::string filename);

	/** 
	Set a joint tag to the bone of bone_name.
	*/
	void SetJointTag(std::string bone_name, ml::JointTag t);
	std::string GetBoneName(ml::JointTag t) const;


	/** 
	Discard Untagged
	*/
	void DiscardUntaggedBones();

	/**
	*/
	void BuildSkeleton();

	void UpdateCharacterPose();


	//////////////////////////////////////////////////
	// Pose
	void Retarget(const ml::Posture &in, bool limb_ik=false, bool head_ik=false);

	void Draw(GL_Renderer* r);

	double CalculScaleTo(const ml::Body *body) const;

protected:
	void ReplaceBone(std::string target_bone_name, std::string src_bone_name);


	ml::JointTag GetJointTagByBoneName(std::string bone_name) const;
	const GL_SceneNode* GetBoneNodeByJointTag(ml::JointTag c) const;
	const GL_SceneNode* GetNearestTaggedParentBoneNode(const GL_SceneNode* c) const;
	ml::JointTag GetNearestTaggedParentTag(ml::JointTag c) const;
	

	void SetBoneMatrix(int bone_id, cml::matrix44d m);


	//////////////////////////////////////////////////
	// Pose

protected:
	GL_SceneRoot character_model_root_;
	ml::Body skeleton_;
	ml::Posture posture_;

	std::map<std::string, int> bone_name_to_id_map_;
	std::map<int, std::string> bone_id_to_name_map_;
	std::map<std::string, const GL_SceneNode*> bone_name_to_node_map_;	// hierarchy info

	std::map<int, cml::matrix44d> bone_id_to_offset_map_;

	std::map<ml::JointTag, int> joint_tag_to_bone_id_map_;
	std::map<int, ml::JointTag> bone_id_to_joint_tag_map_;

	
};






};





