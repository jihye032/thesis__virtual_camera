
#pragma once
#include "BaseLib/CmlExt/CmlExt.h"
#include <map>
#include <set>


namespace mg
{

class GL_Material;
class GL_Mesh;
class GL_VAO;

class GL_RenderableObj
{
	friend class GL_Renderer;
	friend class GL_ResourceManager;
	friend class GL_Character;

private:
	GL_RenderableObj();
public:
	~GL_RenderableObj();

	void Clear();

	void Initialize(GL_VAO *vao, GL_Material *material = 0);
	void Initialize(GL_Mesh *mesh, GL_Material *material = 0);

	void SetBoneMatrix(int bone_id, cml::matrix44d m);
	void SetBoneOffsetMatrix(int bone_id, cml::matrix44d m);
	void SetBoneName(int bone_id, std::string n);

	const cml::matrix44d& GetBoneMatrix(int bone_id) const;
	const cml::matrix44d& GetBoneOffsetMatrix(int bone_id) const;
	std::string GetBoneName(int bone_id) const;

	const std::set<int>& using_bone_matrix_id_set() const { return using_bone_matrix_ids_; }



	void flag_skinning(bool f) { flag_skinning_ = f; }
	void flag_material(bool f) { flag_material_ = f; }

	GL_VAO* vao() const;
	GL_Mesh* mesh() const { return mesh_; }
	GL_Material* material() const { return material_; }
	
	bool flag_skinning() const { return flag_skinning_; }
	bool flag_material() const { return flag_material_; }

	std::string name() const { return name_; }


protected:
	void ReplaceBone(int target_bone_id, int new_bone_id, std::string new_bone_name, cml::matrix44d new_bone_offset);


protected:
	std::string name_;

	GL_Material *material_;
	GL_Mesh *mesh_;
	GL_VAO *vao_;


	bool flag_skinning_;
	bool flag_material_;

	std::set<int> using_bone_matrix_ids_;

	// <bone_id, bone_name>
	std::map<int, std::string> bone_names_;

	// <bone_id, offset_matrix>
	std::map<int, cml::matrix44d> bone_offset_matrices_;
	
	// <bone_id, pose_matrix>
	std::map<int, cml::matrix44d> bone_matrices_;
};


class GL_RenderableObjGroup
{
	friend class GL_Renderer;
	friend class GL_ResourceManager;
private:
	GL_RenderableObjGroup();

public:
	virtual ~GL_RenderableObjGroup();

	void AddRendereableObj(const GL_RenderableObj *obj);
	void AddRendereableObj(const GL_RenderableObj *obj, const cml::matrix44d &global_t);
	void RemoveRendereableObj(const GL_RenderableObj *obj);

protected:
	std::vector<cml::matrix44d> global_t_of_objs_;
	std::vector<const GL_RenderableObj *> renderable_objs_;

};


};