#ifdef ODE_EXT

#include "BaseLib/OdeExt/URDF.h"
#include <sstream>
#include "BaseLib/CmlExt/CmlExt.h"
#include <set>
#include <fstream>
#include <string>
#include <vector>
#include <algorithm>
#include <iomanip>
#include <BaseLib/Algorithm/Signal1D.h>
#include <BaseLib/Algorithm/Signal3D.h>
#include <BaseLib/Geometry/Line.h>



static double safe_double(double val)
{
	#ifdef WIN32
	if (_isnan(val) || !_finite(val))
		return 0.0;
	else
		return val;
	#else
	if (isnan(val) || isinf(val))
		return 0.0;
	else
		return val;

	#endif
};	




static std::string trim(const std::string& str,
                 const std::string& whitespace = " \r\n\t")
{
	const auto strBegin = str.find_first_not_of(whitespace);
	if (strBegin == std::string::npos)
    return ""; // no content

    const auto strEnd = str.find_last_not_of(whitespace);
    const auto strRange = strEnd - strBegin + 1;

    return str.substr(strBegin, strRange);
}


Urdf::~Urdf()
{
	Clear();
}

void
Urdf::Clear()
{
	for ( unsigned int i=0; i<(int)joints_.size(); i++ )
	{
		delete  joints_[i];
	}
	joints_.clear();

	for ( unsigned int i=0; i<(int)links_.size(); i++ )
	{
		delete  links_[i];
	}
	links_.clear();

	joint_name_to_id_.clear();
	link_name_to_id_.clear();
}


UrdfJoint* 
Urdf::GetParentJointOfJoint(int i) const
{
	if ( i < 0 || (int)joints_.size() <= i ) return 0;
	else return GetParentJointOfJoint( joints_[i] );
}



UrdfJoint* 
Urdf::GetParentJointOfJoint(std::string joint_name) const
{
	UrdfJoint *j = joint(joint_name);
	if ( j==0 ) return 0;
	else return GetParentJointOfJoint(j);
}

UrdfJoint* 
Urdf::GetParentJointOfJoint(UrdfJoint* j) const
{
	UrdfLink *parent_link = link(j->parent_link_name_);
	if ( parent_link == 0 ) return 0;

	UrdfJoint *parent_joint = joint(parent_link->parent_joint_name_);
	
	return parent_joint;
}


UrdfLink* 
Urdf::GetChildLinkOfJoint(int i) const
{
	if ( i < 0 || (int)joints_.size() <= i ) return 0;
	else return GetChildLinkOfJoint( joints_[i] );
}



UrdfLink* 
Urdf::GetChildLinkOfJoint(std::string joint_name) const
{
	UrdfJoint *j = joint(joint_name);
	if ( j==0 ) return 0;

	return GetChildLinkOfJoint(j);
}

UrdfLink* 
Urdf::GetChildLinkOfJoint(UrdfJoint* j) const
{
	UrdfLink *child_link = link(j->child_link_name_);
	return  child_link;
}




::vector
Urdf::GetGlobalJointPosition(int i) const
{
	if ( i < 0 || (int)joints_.size() <= (int)i ) return ::vector(0, 0, 0);

	return GetGlobalJointPosition(joints_[i]);
}



::vector
Urdf::GetGlobalJointPosition(std::string joint_name) const
{
	return GetGlobalJointPosition(joint(joint_name));
}

::vector
Urdf::GetGlobalJointPosition(UrdfJoint* j) const
{
	if ( j==0 ) return ::vector(0, 0, 0);

	UrdfJoint *joint = j;
	
	::vector t = joint->translation_;

	joint = GetParentJointOfJoint(joint);
	while ( joint != 0 )
	{
		t = rotate( joint->orientation_, t );
		t += joint->translation_;
		joint = GetParentJointOfJoint(joint);
	}

	return t;
}


::quater
Urdf::GetGlobalJointRotation(int i) const
{
	if ( i < 0 || (int)joints_.size() <= (int)i ) return ::quater(1, 0, 0, 0);

	return GetGlobalJointRotation(joints_[i]);
}



::quater
Urdf::GetGlobalJointRotation(std::string joint_name) const
{
	return GetGlobalJointRotation(joint(joint_name));
}

::quater
Urdf::GetGlobalJointRotation(UrdfJoint* j) const
{
	if ( j==0 ) return ::quater(1, 0, 0, 0);

	UrdfJoint *joint = j;
	
	::quater q = joint->orientation_;

	joint = GetParentJointOfJoint(joint);
	while ( joint != 0 )
	{
		q = joint->orientation_ * q;
		joint = GetParentJointOfJoint(joint);
	}

	return q;
}














/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Load from file
mg::PrimitiveShape*
Urdf::LoadURDFCollision(TiXmlNode* colli_node)
{
	mg::PrimitiveShape* shape = 0;
	::vector local_ori(0, 0, 0);
	::vector local_tra(0, 0, 0);

	TiXmlNode* pChild;
	for ( pChild = colli_node->FirstChild(); pChild != 0; pChild = pChild->NextSibling() ) 
	{
		if ( pChild->Type() == TiXmlNode::TINYXML_ELEMENT )
		{
			if ( strncmp(pChild->Value(), "origin", 6) == 0 )
			{
				const char *tmp_str;
				tmp_str = pChild->ToElement()->Attribute("rpy");
				if ( tmp_str != 0 )
				{
					std::stringstream tmp_sstr(tmp_str);
					tmp_sstr >> local_ori[0];
					tmp_sstr >> local_ori[1];
					tmp_sstr >> local_ori[2];
				}
				tmp_str = pChild->ToElement()->Attribute("xyz");
				if ( tmp_str != 0 )
				{
					std::stringstream tmp_sstr(tmp_str);
					tmp_sstr >> local_tra[0];
					tmp_sstr >> local_tra[1];
					tmp_sstr >> local_tra[2];
				}
			}
			else if ( strncmp(pChild->Value(), "geometry", 8) == 0 )
			{
				TiXmlNode* pChildChild;
				for ( pChildChild = pChild->FirstChild(); pChildChild != 0; pChildChild = pChildChild->NextSibling() )
				{
					if ( strncmp(pChildChild->Value(), "cylinder", 8) == 0 )
					{
						double capsule_len = atof( pChildChild->ToElement()->Attribute("length") );
						double capsule_radi = atof( pChildChild->ToElement()->Attribute("radius") );

						shape = new mg::PrimitiveCylinder(capsule_len, capsule_radi);
						//shape->RotateLocally( exp(x_axis*M_PI/4) );
						((mg::PrimitiveCylinder*)shape)->direction(2);	// the cylinder is laid on z_axix by definition of urdf formation.
					}
					else if ( strncmp(pChildChild->Value(), "box", 3) == 0 )
					{
						const char *tmp_str = pChildChild->ToElement()->Attribute("size");
						double w, h, d;
						std::stringstream tmp_sstr(tmp_str);
						tmp_sstr >> w;
						tmp_sstr >> h;
						tmp_sstr >> d;

						shape = new mg::PrimitiveBox(w, h, d);
						//shape->RotateLocally( exp(x_axis*M_PI/4) );
					}
					else if ( strncmp(pChildChild->Value(), "sphere", 3) == 0 )
					{
						double sphere_r = atof( pChildChild->ToElement()->Attribute("radius") );

						shape = new mg::PrimitiveSphere(sphere_r);
					}
					
				}

				if ( shape != 0 ) break;
			}
		}
	}

	if ( shape )
	{
		shape->RotateLocally( EulerAngle2Quater(local_ori) );
		shape->TranslateLocally( local_tra );
	}
	return shape;
}

void
Urdf::LoadURDFVisual(TiXmlNode* colli_node, UrdfLink* link)
{
	::vector local_orien(0, 0, 0);
	::vector local_trans(0, 0, 0);
	::vector local_scaling(1, 1, 1);
	std::string obj_file_name;
	::vector color(0, 0, 0);
	double alpha;
	mg::Mesh mesh_model;


	TiXmlNode* pChild;
	for ( pChild = colli_node->FirstChild(); pChild != 0; pChild = pChild->NextSibling() ) 
	{
		if ( pChild->Type() == TiXmlNode::TINYXML_ELEMENT )
		{
			if ( strncmp(pChild->Value(), "origin", 6) == 0 )
			{
				const char *tmp_str;
				tmp_str = pChild->ToElement()->Attribute("rpy");
				if ( tmp_str != 0 )
				{
					std::stringstream tmp_sstr(tmp_str);
					tmp_sstr >> local_orien[0];
					tmp_sstr >> local_orien[1];
					tmp_sstr >> local_orien[2];

					// This rotation will be applied to the 'mesh_obj' later in this fuction. 
				}
				tmp_str = pChild->ToElement()->Attribute("xyz");
				if ( tmp_str != 0 )
				{
					std::stringstream tmp_sstr(tmp_str);
					tmp_sstr >> local_trans[0];
					tmp_sstr >> local_trans[1];
					tmp_sstr >> local_trans[2];
					// This translation will be applied to the 'mesh_obj' later in this fuction. 
				}
			}
			else if ( strncmp(pChild->Value(), "geometry", 8) == 0 )
			{
				TiXmlNode* pChildChild;
				for ( pChildChild = pChild->FirstChild(); pChildChild != 0; pChildChild = pChildChild->NextSibling() )
				{
					if ( strncmp(pChildChild->Value(), "mesh", 4) == 0 )
					{
						obj_file_name = pChildChild->ToElement()->Attribute("filename");
						// I only saved the filename of the mesh model.
						// It will be loaded later in this function. 

						const char *tmp_str;
						tmp_str = pChildChild->ToElement()->Attribute("scale");
						if ( tmp_str != 0 )
						{
							std::stringstream tmp_sstr(tmp_str);
							tmp_sstr >> local_scaling[0];
							tmp_sstr >> local_scaling[1];
							tmp_sstr >> local_scaling[2];
						}
					}
					else if ( strncmp(pChildChild->Value(), "cylinder", 8) == 0 )
					{
						double capsule_len = atof( pChildChild->ToElement()->Attribute("length") );
						double capsule_radi = atof( pChildChild->ToElement()->Attribute("radius") );

						mesh_model.CreateCylinder(capsule_len, capsule_radi);
						mesh_model.RotateVertices( exp(x_axis*M_PI/4) ); // the cylinder is laid on z_axix by definition of urdf formation.
					}
					else if ( strncmp(pChildChild->Value(), "box", 3) == 0 )
					{
						const char *tmp_str = pChildChild->ToElement()->Attribute("size");
						double w, h, d;
						std::stringstream tmp_sstr(tmp_str);
						tmp_sstr >> w;
						tmp_sstr >> h;
						tmp_sstr >> d;

						mesh_model.CreateBox(w, h, d);
					}
					else if ( strncmp(pChildChild->Value(), "sphere", 3) == 0 )
					{
						double sphere_r = atof( pChildChild->ToElement()->Attribute("radius") );

						mesh_model.CreateSphere(sphere_r);
					}
				}
			}
			else if ( strncmp(pChild->Value(), "material", 8) == 0 )
			{
				TiXmlNode* pChildChild;
				for ( pChildChild = pChild->FirstChild(); pChildChild != 0; pChildChild = pChildChild->NextSibling() )
				{
					if ( strncmp(pChildChild->Value(), "color", 4) == 0 )
					{
						const char *tmp_str;
						tmp_str = pChildChild->ToElement()->Attribute("rgba");
						std::stringstream tmp_sstr(tmp_str);
						tmp_sstr >> color[0];
						tmp_sstr >> color[1];
						tmp_sstr >> color[2];
						tmp_sstr >> alpha;
					}
				}
			}
		}
	}

	// Load Mesh Model from the File.
	if ( !obj_file_name.empty() )
	{
		// File Name converting
		{
			if ( obj_file_name.compare(0, 10, "package://") == 0 )
			{
				obj_file_name.erase(obj_file_name.begin(), obj_file_name.begin()+10);
			}

			std::size_t e = obj_file_name.find_last_of(".");
			if ( e != std::string::npos )
			{
				obj_file_name.erase(obj_file_name.begin()+e, obj_file_name.end());
			}

			obj_file_name = "data/" + obj_file_name + ".obj";
		}
		mesh_model.ReadObjFormatFile(obj_file_name);
	}

	mesh_model.ScaleVertices( local_scaling );
	mesh_model.RotateVertices( EulerAngle2Quater(local_orien) );
	mesh_model.TranslateVertices( local_trans );

	
	// Assign
	if ( link->mesh_model_ == 0 ) 
		link->mesh_model_= new mg::SolidObject();
	link->mesh_model_->editable_mesh()->Merge(mesh_model);
	link->color_ = color;
	link->alpha_ = alpha;
}

void
Urdf::LoadURDFInertial(TiXmlNode* colli_node, UrdfLink* link)
{
	double mass = 0;
	::vector rpy(0, 0, 0);
	::vector center_of_mass(0, 0, 0);
	double ixx=1, ixy=0, ixz=0, iyy=1, iyz=0, izz=1;

	TiXmlNode* pChild;
	for ( pChild = colli_node->FirstChild(); pChild != 0; pChild = pChild->NextSibling() ) 
	{
		if ( pChild->Type() == TiXmlNode::TINYXML_ELEMENT )
		{
			if ( strncmp(pChild->Value(), "mass", 4) == 0 )
			{
				const char *tmp_str;
				tmp_str = pChild->ToElement()->Attribute("value");
				if ( tmp_str != 0 )
				{
					mass = atof(tmp_str);
				}
			}
			else if ( strncmp(pChild->Value(), "origin", 6) == 0 )
			{
				const char *tmp_str;
				tmp_str = pChild->ToElement()->Attribute("rpy");
				if ( tmp_str != 0 )
				{
					std::stringstream tmp_sstr(tmp_str);
					tmp_sstr >> rpy[0];
					tmp_sstr >> rpy[1];
					tmp_sstr >> rpy[2];
				}
				tmp_str = pChild->ToElement()->Attribute("xyz");
				if ( tmp_str != 0 )
				{
					std::stringstream tmp_sstr(tmp_str);
					tmp_sstr >> center_of_mass[0];
					tmp_sstr >> center_of_mass[1];
					tmp_sstr >> center_of_mass[2];
				}
			}
			else if ( strncmp(pChild->Value(), "inertia", 8) == 0 )
			{
				const char *tmp_str;
				
				tmp_str = pChild->ToElement()->Attribute("ixx");
				if ( tmp_str != 0 ) ixx = atof(tmp_str);

				tmp_str = pChild->ToElement()->Attribute("ixy");
				if ( tmp_str != 0 ) ixy = atof(tmp_str);

				tmp_str = pChild->ToElement()->Attribute("ixz");
				if ( tmp_str != 0 ) ixz = atof(tmp_str);

				tmp_str = pChild->ToElement()->Attribute("iyy");
				if ( tmp_str != 0 ) iyy = atof(tmp_str);

				tmp_str = pChild->ToElement()->Attribute("iyz");
				if ( tmp_str != 0 ) iyz = atof(tmp_str);

				tmp_str = pChild->ToElement()->Attribute("izz");
				if ( tmp_str != 0 ) izz = atof(tmp_str);

				link->flag_inertia_set_ = true;
			}
		}
	}

	// Assign
	link->mass_ = mass;
	link->center_of_mass_ = center_of_mass;

	link->inertia_[0][0] = ixx;
	link->inertia_[0][1] = ixy;
	link->inertia_[0][2] = ixz;

	link->inertia_[1][0] = ixy;
	link->inertia_[1][1] = iyy;
	link->inertia_[1][2] = iyz;

	link->inertia_[2][0] = ixz;
	link->inertia_[2][1] = iyz;
	link->inertia_[2][2] = izz;
}

void
Urdf::LoadURDFLink(TiXmlNode* link_node)
{
	if ( !link_node ) return;

	// Name
	const char *name_c_str = link_node->ToElement()->Attribute("name");
	if ( name_c_str == 0 )
	{
		return;
	}
	std::string name_str(name_c_str);
	name_str = trim(name_str);

	UrdfLink* cur_link = 0;
	if ( link_name_to_id_.find(name_str) == link_name_to_id_.end() )
	{
		link_name_to_id_[name_str] = (int)links_.size();
		links_.push_back(new UrdfLink);
		cur_link = links_.back();
	}
	else
	{
		cur_link= links_[ link_name_to_id_[name_str] ];
	}
	cur_link->name_ = name_str;
	cur_link->bounding_volume_ = 0;

	TiXmlNode* pChild;
	for ( pChild = link_node->FirstChild(); pChild != 0; pChild = pChild->NextSibling()) 
	{
		if ( pChild->Type() == TiXmlNode::TINYXML_ELEMENT )
		{
			if ( strncmp(pChild->Value(), "collision", 9) == 0 )
			{
				
				if ( strncmp(pChild->ToElement()->FirstAttribute()->Value(), "default", 7) == 0 )
				{
					cur_link->bounding_volume_ = LoadURDFCollision(pChild);
				}
				else
				{
					cur_link->other_bounding_volumes_.push_back(LoadURDFCollision(pChild));
				}
			}

			else if ( strncmp(pChild->Value(), "visual", 6) == 0 )
			{
				LoadURDFVisual(pChild, cur_link);
			}

			else if ( strncmp(pChild->Value(), "inertial", 8) == 0 )
			{
				LoadURDFInertial(pChild, cur_link);
			}
		}
	}
}

void
Urdf::LoadURDFJoint(TiXmlNode* joint_node)
{
	if ( !joint_node ) return;



	// Name
	const char *name_c_str = joint_node->ToElement()->Attribute("name");
	if ( name_c_str == 0 )
	{
		return;
	}
	std::string name_str(name_c_str);
	name_str = trim(name_str);

	UrdfJoint *cur_joint = 0;
	if ( joint_name_to_id_.find(name_str) == joint_name_to_id_.end() )
	{
		joint_name_to_id_[name_str] = (int)joints_.size();
		joints_.push_back(new UrdfJoint);
		cur_joint = joints_.back();
	}
	else
	{
		cur_joint = joints_[joint_name_to_id_[name_str]];
	}
	cur_joint->name_ = name_str;

	// Type
	const char *type_c_str = joint_node->ToElement()->Attribute("type");
	if ( type_c_str != 0 )
	{
		cur_joint->type_ = type_c_str;
	}


	TiXmlNode* pChild;
	for ( pChild = joint_node->FirstChild(); pChild != 0; pChild = pChild->NextSibling()) 
	{
		if ( pChild->Type() == TiXmlNode::TINYXML_ELEMENT )
		{
			if ( strncmp(pChild->Value(), "origin", 6) == 0 )
			{
				const char *tmp_str;
				tmp_str = pChild->ToElement()->Attribute("rpy");
				if ( tmp_str != 0 )
				{
					::vector rpy;
					std::stringstream tmp_sstr(tmp_str);
					tmp_sstr >> rpy[0];
					tmp_sstr >> rpy[1];
					tmp_sstr >> rpy[2];
					cur_joint->orientation_ = EulerAngle2Quater(rpy);
				}
				tmp_str = pChild->ToElement()->Attribute("xyz");
				if ( tmp_str != 0 )
				{
					std::stringstream tmp_sstr(tmp_str);
					tmp_sstr >> cur_joint->translation_[0];
					tmp_sstr >> cur_joint->translation_[1];
					tmp_sstr >> cur_joint->translation_[2];
				}
			}
			else if ( strncmp(pChild->Value(), "axis", 4) == 0 )
			{
				const char *tmp_str;
				tmp_str = pChild->ToElement()->Attribute("xyz");
				if ( tmp_str != 0 )
				{
					std::stringstream tmp_sstr(tmp_str);
					tmp_sstr >> cur_joint->axis_[0];
					tmp_sstr >> cur_joint->axis_[1];
					tmp_sstr >> cur_joint->axis_[2];
				}
			}
			else if ( strncmp(pChild->Value(), "parent", 6) == 0 )
			{
				const char *tmp_str;
				tmp_str = pChild->ToElement()->Attribute("link");
				cur_joint->parent_link_name_ = tmp_str;
			}
			else if ( strncmp(pChild->Value(), "child", 6) == 0 )
			{
				const char *tmp_str;
				tmp_str = pChild->ToElement()->Attribute("link");
				cur_joint->child_link_name_ = tmp_str;
			} 
			else if ( strncmp(pChild->Value(), "limit", 5) == 0 )
			{
				const char *tmp_str;

				cur_joint->limit_effort_ = 0;
				cur_joint->limit_lower_ = 0;
				cur_joint->limit_upper_ = 0;
				cur_joint->limit_velocity_ = 0;

				tmp_str = pChild->ToElement()->Attribute("effort");
				if ( tmp_str != 0 )
					cur_joint->limit_effort_ = atof( tmp_str );

				tmp_str = pChild->ToElement()->Attribute("lower");
				if ( tmp_str != 0 )
					cur_joint->limit_lower_ = atof( tmp_str );

				tmp_str = pChild->ToElement()->Attribute("upper");
				if ( tmp_str != 0 )
					cur_joint->limit_upper_ = atof( tmp_str );

				tmp_str = pChild->ToElement()->Attribute("velocity");
				if ( tmp_str != 0 )
					cur_joint->limit_velocity_ = atof( tmp_str );
			}
		}
	}
}

void 
Urdf::LoadURDF( TiXmlNode* pParent )
{
	if ( !pParent ) return;

	TiXmlNode* pChild;
	for ( pChild = pParent->FirstChild(); pChild != 0; pChild = pChild->NextSibling()) 
	{
		if ( pChild->Type() == TiXmlNode::TINYXML_ELEMENT )
		{
			if ( strncmp(pChild->Value(), "link", 4) == 0 )
			{
				LoadURDFLink( pChild );
			}
			else if ( strncmp(pChild->Value(), "joint", 5) == 0 )
			{
				LoadURDFJoint( pChild );
			}
			else
			{
				LoadURDF(pChild);
			}
		}
	}
}

void
Urdf::LoadURDF(std::string filename)
{
	Clear();
	TiXmlDocument doc(filename.c_str());
	doc.LoadFile();
	LoadURDF(&doc);

	if ( joint("right_sandia_hand_joint") != 0 )
	{
		std::cout << joint("right_sandia_hand_joint")->orientation_ << std::endl;//
	}

	// set child and parent joint of links
	{
		for ( unsigned int i=0; i<joints_.size(); i++ )
		{
			UrdfLink *p_link = link(joints_[i]->parent_link_name_);
			UrdfLink *c_link = link(joints_[i]->child_link_name_);
			
			if ( p_link ) p_link->child_joint_names_.push_back(joint_name(i));
			if ( c_link ) c_link->parent_joint_name_ = joint_name(i);
		}
	}

	RemoveFixedJoints();
}



void
Urdf::RemoveFixedJoints()
{
	// A fixed joint is a joint that is not be angle to rotate, so it just ties two rigid bodys as on rigid body.
	// It just increases the complexity of the model.
	// The more serious problem is that a fixed jonit makes ODE engine seriously slower and unstable. 
	// So I will remove all fixed joints and merge both linked body parts as one body part. 

	for ( unsigned int joint_id=0; joint_id<joints_.size(); joint_id++ )
	{
		if ( joints_[joint_id]->type_.compare("fixed") == 0 
			)
		{
			UrdfJoint *fixed_joint = joints_[joint_id];
			int a_link_id = link_id(fixed_joint->parent_link_name_);
			int b_link_id = link_id(fixed_joint->child_link_name_);
			UrdfLink *a_link = link(a_link_id);
			UrdfLink *b_link = link(b_link_id);

			if ( a_link == 0 ) continue;
			if ( b_link == 0 ) continue;

			// Merge both Links
			{
				// Merge Bounding Volumes.
				if ( b_link->bounding_volume_ )
				{
					mg::PrimitiveShape *s = b_link->bounding_volume_->CreateClone();
					s->RotateLocally(fixed_joint->orientation_);
					s->TranslateLocally(fixed_joint->translation_);
					a_link->other_bounding_volumes_.push_back(s);
				}
				for ( unsigned int o=0; o<b_link->other_bounding_volumes_.size(); o++ )
				{
					mg::PrimitiveShape *s = b_link->other_bounding_volumes_[o]->CreateClone();
					s->RotateLocally(fixed_joint->orientation_);
					s->TranslateLocally(fixed_joint->translation_);
					a_link->other_bounding_volumes_.push_back(s);
				}

				// Merge Mesh Models.
				if ( b_link->mesh_model_ )
				{
					b_link->mesh_model_->editable_mesh()->RotateVertices(fixed_joint->orientation_);
					b_link->mesh_model_->editable_mesh()->TranslateVertices(fixed_joint->translation_);
					a_link->mesh_model_->editable_mesh()->Merge(*b_link->mesh_model_->mesh());
				}

				// Merged Center of Mass
				::vector a_com;
				::vector b_com;
				double merged_mass;
				::vector merged_com;
				{
					a_com = a_link->center_of_mass_;
					b_com = rotate(fixed_joint->orientation_, fixed_joint->translation_ + b_link->center_of_mass_);
					
					merged_mass = a_link->mass_+b_link->mass_;
					
					merged_com = ( a_link->mass_*a_com + b_link->mass_*b_com) / (merged_mass);
				}

				// Merge Inertias
				{
					// http://physics.stackexchange.com/questions/17336/how-do-you-combine-two-rigid-bodies-into-one

					::vector t1 = merged_com - a_com;
					::vector t2 = merged_com - b_com;

					::matrix J1(-1*(SQR(t1.y())+SQR(t1.z())), t1.x()*t1.y(), t1.x()*t1.z(),
							t1.x()*t1.y(), -1*(SQR(t1.x())+SQR(t1.z())), t1.y()*t1.z(),
							t1.x()*t1.z(), t1.y()*t1.z(), -1*(SQR(t1.x())+SQR(t1.y()))
							);
					::matrix J2(-1*(SQR(t2.y())+SQR(t2.z())), t2.x()*t2.y(), t2.x()*t2.z(),
							t2.x()*t2.y(), -1*(SQR(t2.x())+SQR(t2.z())), t2.y()*t2.z(),
							t2.x()*t2.z(), t2.y()*t2.z(), -1*(SQR(t2.x())+SQR(t2.y()))
							);

					::matrix b_rotation = Quater2Matrix(fixed_joint->orientation_);
					::matrix merged_inertia = a_link->inertia_ + a_link->mass_*J1
											+ b_rotation*b_link->inertia_*b_rotation.transpose() + b_link->mass_*J2;

					a_link->inertia_ = merged_inertia;
					a_link->center_of_mass_ = merged_com;
					a_link->mass_ = merged_mass;
				}

				
				
				// Reget the relations with joints
				a_link->child_joint_names_.erase(
					std::find(a_link->child_joint_names_.begin(), a_link->child_joint_names_.end(), fixed_joint->name_)
					);

				for ( unsigned int c=0; c<b_link->child_joint_names_.size(); c++ )
				{
					UrdfJoint *child_joint = joint(b_link->child_joint_names_[c]);
					if ( !child_joint ) continue;

					child_joint->parent_link_name_ = a_link->name_;
					child_joint->orientation_ = fixed_joint->orientation_ * child_joint->orientation_;// * fixed_joint->orientation_;
					child_joint->translation_ = fixed_joint->translation_ 
										+ rotate(fixed_joint->orientation_, child_joint->translation_);
					a_link->child_joint_names_.push_back(child_joint->name_);
				}

				// Delete the fixed Joint and B_link.
				joint_name_to_id_.erase(joint_name_to_id_.find(fixed_joint->name_));
				link_name_to_id_.erase(link_name_to_id_.find(b_link->name_));
				delete fixed_joint;
				delete b_link;
				joints_[joint_id] = 0;
				links_[b_link_id] = 0;
			}
		}
	}


	// Compress the joints_ array
	for ( unsigned int joint_id=0; joint_id<joints_.size(); joint_id++ )
	{
		if ( joints_[joint_id] == 0 )
		{
			joints_.erase( joints_.begin()+joint_id );
			joint_id--;
		}
	}
	for ( unsigned int joint_id=0; joint_id<joints_.size(); joint_id++ )
	{
		joint_name_to_id_[joints_[joint_id]->name_] = joint_id;
	}

	// Compress the links_ array
	for ( unsigned int link_id=0; link_id<links_.size(); link_id++ )
	{
		if ( links_[link_id] == 0 )
		{
			links_.erase( links_.begin()+link_id );
			link_id--;
		}
	}
	for ( unsigned int link_id=0; link_id<links_.size(); link_id++ )
	{
		link_name_to_id_[links_[link_id]->name_] = link_id;
	}

}

int
Urdf::degree_of_freedom() const
{
	int count = 0;
	for ( unsigned int i=0; i<flag_active_joints_.size(); i++ )
	{
		if ( flag_active_joints_[i] ) count++;
	}

	return count;
}

bool
Urdf::IsAncestorOf(int ancestor_id, int child_id) const
{

	while ( child_id >= 0 )
	{
		if (child_id==ancestor_id) return true;
		child_id = GetParentJointIdOfJoint(child_id);
	}

	return false;
}

/*
void
Urdf::Rotate(quater q)
{
	for ( unsigned int i=0; i<joints_.size(); i++ )
	{
		//joints_[i]->orientation_ =  joints_[i]->orientation_ * q;
		joints_[i]->axis_ = rotate(q, joints_[i]->axis_).normalize();
		joints_[i]->translation_ = rotate(q, joints_[i]->translation_);
	}

	for ( unsigned int i=0; i<links_.size(); i++ )
	{
		quater link_r = q;
		UrdfJoint *parent_joint = joint(links_[i]->parent_joint_name_);
		if ( parent_joint != 0 ) 
		{
			link_r = parent_joint->orientation_.inverse() * link_r;// *parent_joint->orientation_;
		}

		if ( links_[i]->mesh_model_ )
		{
			links_[i]->mesh_model_->editable_mesh()->RotateVertices(link_r);
		}

		if ( links_[i]->bounding_volume_ != 0 )
		{
			links_[i]->bounding_volume_->RotateLocally(q);
		}

		for ( unsigned int j=0; j<links_[i]->other_bounding_volumes_.size(); j++ )
		{
			links_[i]->other_bounding_volumes_[j]->RotateLocally(q);
		}

		links_[i]->center_of_mass_ = rotate(q, links_[i]->center_of_mass_);
		matrix r = Quater2Matrix(q);
		links_[i]->inertia_ = r * links_[i]->inertia_ * r.transpose();
		// links_[i]->inertia_ = r.transpose() * links_[i]->inertia_ * r;
	}
}
*/

void
Urdf::Scale(double s)
{
	for ( unsigned int i=0; i<joints_.size(); i++ )
	{
		joints_[i]->translation_ *= s;
	}

	for ( unsigned int i=0; i<links_.size(); i++ )
	{
		if ( links_[i]->mesh_model_ )
		{
			links_[i]->mesh_model_->editable_mesh()->ScaleUniformlyVertices(s);
		}

		if ( links_[i]->bounding_volume_ != 0 )
		{
			links_[i]->bounding_volume_->ScaleShape(s);
			links_[i]->bounding_volume_->local_translation(s*links_[i]->bounding_volume_->local_translation());
		}

		for ( unsigned int j=0; j<links_[i]->other_bounding_volumes_.size(); j++ )
		{
			links_[i]->other_bounding_volumes_[j]->ScaleShape(s);
			links_[i]->other_bounding_volumes_[j]->local_translation(
				s*links_[i]->other_bounding_volumes_[j]->local_translation());
		}

		links_[i]->center_of_mass_ *= s;
		links_[i]->inertia_ = links_[i]->inertia_ * (s*s);
	}
}


#endif
