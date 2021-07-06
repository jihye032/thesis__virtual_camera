#ifdef ODE_EXT

#pragma once

#include <string>
#include "BaseLib/CmlExt/CmlExt.h"
#include <map>
#include <vector>
#include "BaseLib/tinyxml/tinyxml.h"
#include "BaseLib/Geometry/PrimitiveShape.h"
#include "PmQm/qmPolygon2D.h"

class UrdfJoint
{
public:
	UrdfJoint()
	{
		//angle_ = 0;

		// Just for stability.
		axis_ = ::vector(1, 0, 0);
		translation_ = ::vector(0, 0, 0);
		orientation_ = ::quater(1, 0, 0, 0);

		limit_effort_ = 0;
		limit_lower_ = 0;
		limit_upper_ = 0;
		limit_velocity_ = 0;
	}

	std::string name_;
	std::string type_;

	std::string parent_link_name_;
	std::string child_link_name_;

	::vector axis_;
	::vector translation_;
	::quater orientation_;

	double limit_effort_;
	double limit_lower_;
	double limit_upper_;
	double limit_velocity_;
};

class UrdfLink
{
public:
	UrdfLink()
	{
		bounding_volume_ = 0;
		mesh_model_ = 0;

		color_.setValue(0, 0, 0);
		alpha_ = 1.0;

		mass_ = 0;
		center_of_mass_.setValue(0, 0, 0);
		inertia_ = matrix(1, 0, 0, 0, 1, 0, 0, 0, 1);
		flag_inertia_set_ = false;
	}
	~UrdfLink()
	{
		if ( bounding_volume_ ) 
			delete bounding_volume_;
		bounding_volume_ = 0;

		if ( mesh_model_ )
			delete mesh_model_;
		mesh_model_ = 0;
	}

	std::string name_;

	mg::PrimitiveShape* bounding_volume_;
	std::vector<mg::PrimitiveShape*> other_bounding_volumes_; 
	mg::SolidObject *mesh_model_;

	bool flag_inertia_set_;
	double mass_;
	::vector center_of_mass_;
	::matrix inertia_;


	std::vector<std::string> child_joint_names_;
	std::string parent_joint_name_;

	::vector color_;
	double alpha_;
		
};


class Urdf
{
public:
	~Urdf();
	void Clear();
	void LoadURDF(std::string filename);

	int num_joints() const { return joints_.size(); }
	int num_active_joints() const { return 28; }
	std::string joint_name(int i) const { return joints_[i]->name_; }
	int joint_id(UrdfJoint* joint) const       { return joint_id(joint->name_); }
	int joint_id(std::string j_name) const      { return ( joint_name_to_id_.find(j_name) == joint_name_to_id_.end() ) ? -1 : joint_name_to_id_.find(j_name)->second; }
	
	UrdfJoint* joint(int i) const { return joints_[i]; }
	UrdfJoint* joint(std::string j_name) const { int j_id = joint_id(j_name); return ( j_id >= 0 ) ? joint(j_id) : 0; }
	
	
	int num_links() const { return links_.size(); }
	std::string link_name(int i) const { return links_[i]->name_; }
	int link_id(UrdfLink* link) const     { return link_id(link->name_); }
	int link_id(std::string l_name) const  { return ( link_name_to_id_.find(l_name) == link_name_to_id_.end() ) ? -1 : link_name_to_id_.find(l_name)->second; }

	UrdfLink* link(int i) const { return links_[i]; }
	UrdfLink* link(std::string l_name) const { int l_id = link_id(l_name); return (l_id >= 0 ) ? link(l_id) : 0; }

	void Scale(double s);
	//void Rotate(quater q);

	::vector GetGlobalJointPosition(int joint_id) const;
	::vector GetGlobalJointPosition(std::string joint_name) const;
	::vector GetGlobalJointPosition(UrdfJoint* j) const;

	::quater GetGlobalJointRotation(int joint_id) const;
	::quater GetGlobalJointRotation(std::string joint_name) const;
	::quater GetGlobalJointRotation(UrdfJoint* j) const;
	
	UrdfJoint* GetParentJointOfJoint(int joint_id) const;
	UrdfJoint* GetParentJointOfJoint(std::string joint_name) const;
	UrdfJoint* GetParentJointOfJoint(UrdfJoint* j) const;
	int GetParentJointIdOfJoint(int j_id)           const { UrdfJoint *parent_joint=GetParentJointOfJoint(j_id);   return (parent_joint==0) ? -1 : joint_id(parent_joint); }
	int GetParentJointIdOfJoint(std::string j_name) const { UrdfJoint *parent_joint=GetParentJointOfJoint(j_name); return (parent_joint==0) ? -1 : joint_id(parent_joint); }
	int GetParentJointIdOfJoint(UrdfJoint* joint)  const { UrdfJoint *parent_joint=GetParentJointOfJoint(joint);  return (parent_joint==0) ? -1 : joint_id(parent_joint); }
	UrdfLink* GetChildLinkOfJoint(int joint_id) const;
	UrdfLink* GetChildLinkOfJoint(std::string joint_name) const;
	UrdfLink* GetChildLinkOfJoint(UrdfJoint* j) const;
	bool IsAncestorOf(int ancestor_id, int child_id) const;

	int degree_of_freedom() const;
	bool is_active_joint(int joint_id) const { return flag_active_joints_[joint_id]; }

protected:
	void LoadURDF(TiXmlNode* pParent);
	void LoadURDFElement(TiXmlElement* pElement);
	void LoadURDFLink(TiXmlNode* pParent);
	void LoadURDFJoint(TiXmlNode* pParent);
	mg::PrimitiveShape* LoadURDFCollision(TiXmlNode* pParent);
	void LoadURDFVisual(TiXmlNode* pParent, UrdfLink* link);
	void LoadURDFInertial(TiXmlNode* pParent, UrdfLink* link);
	void RearrangeJointOrder();
	void RemoveFixedJoints();

protected:
	std::vector<UrdfJoint*> joints_;
	std::vector<UrdfLink*> links_;
	std::map<std::string, int> joint_name_to_id_;
	std::map<std::string, int> link_name_to_id_;
	
	std::vector<bool> flag_active_joints_;
};

#endif
