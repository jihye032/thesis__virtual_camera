


#include "BaseLib/GL4U/GL_Material.h"
#include "BaseLib/GL4U/GL_Texture.h"



namespace mg
{

GL_Material::GL_Material()
{
	diffuse_color_.set (0.8f, 0.8f, 0.8f, 1.0f);
	specular_color_.set(0.1f, 0.1f, 0.1f, 1.0f);
	ambient_color_.set (0.2f, 0.2f, 0.2f, 1.0f);
	emission_color_.set(0.0f, 0.0f, 0.0f, 1.0f);

	shininess_ = 100.f;
	shininess_strength_ = 1.f;

	flag_use_vertex_color_ = false;
	flag_texture_ = false;
	flag_shading_ = true;
	flag_flat_shading_ = false;
	flag_transparency_ = false;
}

GL_Material::~GL_Material()
{
	for ( unsigned int i=0; i<textures_.size(); i++ )
	{
		delete textures_[i];
	}
}

void
GL_Material::Apply()
{
	for ( unsigned int i=0; i<textures_.size(); i++ )
	{
		textures_[i]->Bind();
	}
}


void
GL_Material::RemoveTexture(GL_Texture *t)
{
	for ( unsigned int i=0; i<textures_.size(); i++ )
	{
		if ( textures_[i] == t )
		{
			textures_.erase(textures_.begin()+i);
			break;
		}
	}
}

};








