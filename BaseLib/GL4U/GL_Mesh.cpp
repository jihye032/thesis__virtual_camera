
#include "BaseLib/GL4U/GL_Mesh.h"
#include "BaseLib/GL4U/GL_VBOVAO.h"



namespace mg
{

GL_Mesh::GL_Mesh()
{
}

GL_Mesh::GL_Mesh(const Mesh &in_mesh)
{
	Mesh::Mesh(in_mesh);
	BuildVAO();
}


GL_Mesh::~GL_Mesh()
{
	Clear();
}

void
GL_Mesh::Clear()
{
}

void
GL_Mesh::BuildVAO()
{
	if ( mesh_type() == GL_Mesh::MT_POLYGONS ) Triangulate();
	vbo_group_ = std::move(std::unique_ptr<GL_VBOGroup>(new GL_VBOGroup));
	vbo_group_->SetByMesh(*this);

	vao_ = std::move(std::unique_ptr<GL_VAO>(new GL_VAO(vbo_group_.get())));
}


void
GL_Mesh::Draw() const
{
	vao_->Draw();
}


};








