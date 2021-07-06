
#include "BaseLib/GL4U/GL_RenderableObj.h"
#include "BaseLib/Geometry/Mesh.h"
#include "BaseLib/GL4U/GL_Material.h"
#include "BaseLib/GL4U/GL_Mesh.h"
#include "BaseLib/GL4U/GL_VBOVAO.h"



namespace mg
{

GL_RenderableObj::GL_RenderableObj()
{
	material_ = 0;
	vao_ = 0;
	mesh_ = 0;
	flag_skinning_ = false;
	flag_material_ = false;
}

GL_RenderableObj::~GL_RenderableObj()
{
	Clear();
}

void
GL_RenderableObj::Clear()
{
	using_bone_matrix_ids_.clear();
	bone_offset_matrices_.clear();
	bone_matrices_.clear();

	vao_ = 0;
	mesh_ = 0;
	material_ = 0;
}

void
GL_RenderableObj::Initialize(GL_VAO *vao, GL_Material *material)
{
	vao_ = vao;
	mesh_ = 0;
	material_ = material;
	if ( material ) flag_material_ = true;
}

void
GL_RenderableObj::Initialize(GL_Mesh *mesh, GL_Material *material)
{
	mesh_ = mesh;
	vao_ = 0;
	material_ = material;
	if ( material ) flag_material_ = true;
}

void
GL_RenderableObj::SetBoneOffsetMatrix(int bone_id, cml::matrix44d m)
{
	using_bone_matrix_ids_.insert(bone_id);
	bone_offset_matrices_[bone_id] = m;
}

void
GL_RenderableObj::SetBoneMatrix(int bone_id, cml::matrix44d m)
{
	using_bone_matrix_ids_.insert(bone_id);
	bone_matrices_[bone_id] = m;
}

void
GL_RenderableObj::SetBoneName(int bone_id, std::string n)
{
	using_bone_matrix_ids_.insert(bone_id);
	bone_names_[bone_id] = n;
}


const cml::matrix44d&
GL_RenderableObj::GetBoneMatrix(int bone_id) const
{
	static cml::matrix44d m;
	m.identity();

	if ( bone_matrices_.find(bone_id) == bone_matrices_.end() )
	{
		return m;
	}
	else
	{
		return bone_matrices_.find(bone_id)->second;
	}
}

const cml::matrix44d&
GL_RenderableObj::GetBoneOffsetMatrix(int bone_id) const
{
	static cml::matrix44d m;
	m.identity();

	if ( bone_offset_matrices_.find(bone_id) == bone_offset_matrices_.end() )
	{
		return m;
	}

	return bone_offset_matrices_.find(bone_id)->second;
}

std::string
GL_RenderableObj::GetBoneName(int bone_id) const
{
	std::string str;
	if ( bone_names_.find(bone_id) == bone_names_.end() )
	{
		str = "";
	}
	else
	{
		str = bone_names_.find(bone_id)->second;
	}

	return str;
}


void 
GL_RenderableObj::ReplaceBone(int target_bone_id, int new_bone_id, std::string new_bone_name, cml::matrix44d new_bone_offset)
{
	if ( using_bone_matrix_ids_.find(target_bone_id) == using_bone_matrix_ids_.end() ) return;


	if ( mesh_ != nullptr )
	{
		int count = 0;
		for ( int i=0; i<mesh_->num_vertices(); i++ )
		{
			for ( int j=0; j<mesh_->max_bone_num_per_vertex(); j++ )
			{
				if ( mesh_->bone_ids(i)[j] == target_bone_id )
				{
					mesh_->bone_ids(i)[j] = new_bone_id;
					count++;
				}
			}
		}

		mesh_->UpdateVAO();
	}

	bone_names_.erase(target_bone_id);
	bone_offset_matrices_.erase(target_bone_id);
	bone_matrices_.erase(target_bone_id);

	bone_names_[new_bone_id] = new_bone_name;
	bone_offset_matrices_[new_bone_id] = new_bone_offset;
	bone_matrices_[new_bone_id] = cml::identity_4x4();

	using_bone_matrix_ids_.erase(target_bone_id);
	using_bone_matrix_ids_.insert(new_bone_id);
}




GL_VAO* 
GL_RenderableObj::vao() const
{ 
	if (mesh_ == 0) 
		return vao_; 
	else 
		return mesh_->vao(); 
}





///////////////////////////////////////////////////////////////////////
// class: GL_RenderableObjGroup
GL_RenderableObjGroup::GL_RenderableObjGroup()
{
}

GL_RenderableObjGroup::~GL_RenderableObjGroup()
{
}

void
GL_RenderableObjGroup::AddRendereableObj(const GL_RenderableObj *obj)
{
	renderable_objs_.push_back(obj);

	cml::matrix44d m;
	m.identity();
	global_t_of_objs_.push_back(m);
}

void
GL_RenderableObjGroup::AddRendereableObj(const GL_RenderableObj *obj, const cml::matrix44d &global_t)
{
	renderable_objs_.push_back(obj);
	global_t_of_objs_.push_back(global_t);
}

void
GL_RenderableObjGroup::RemoveRendereableObj(const GL_RenderableObj *obj)
{
	for ( unsigned int i=0; i<renderable_objs_.size(); i++ )
	{
		if ( renderable_objs_[i] == obj )
		{
			renderable_objs_.erase(renderable_objs_.begin()+i);
			global_t_of_objs_.erase(global_t_of_objs_.begin()+i);
			break;
		}
	}
}

};








