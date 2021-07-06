
#pragma once
#include "BaseLib/Geometry/Mesh.h"
#include "BaseLib/GL4U/GL_VBOVAO.h"
#include <memory>


namespace mg
{

class GL_Mesh : public Mesh
{
	friend class GL_ResourceManager;
private:
	GL_Mesh();
	GL_Mesh(const Mesh &in_mesh);

public:
	virtual ~GL_Mesh();

	virtual void Assign(const Mesh& mesh) override
	{
		Mesh::Assign(mesh);
		UpdateVAO();
	}
	virtual void Merge(const Mesh& mesh) override
	{
		Mesh::Merge(mesh);
		UpdateVAO();
	}

	/*
	This method force to triangulate the mesh model,
	then updates VAO and VBO to be matched with current mesh geomatry.
	After any change of the mesh data, this method should be called.
	*/
	void BuildVAO();
	void UpdateVAO() {BuildVAO();}
	void Draw() const;

	GL_VAO* vao() const { return vao_.get(); }
	std::string name() const { return name_; }

protected:
	void Clear();


protected:
	std::string name_;
	std::unique_ptr<GL_VAO> vao_;
	std::unique_ptr<GL_VBOGroup> vbo_group_;
};


};