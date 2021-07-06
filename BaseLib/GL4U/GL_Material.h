
#pragma once
#include "BaseLib/CmlExt/CmlExt.h"
#include <vector>

namespace mg
{
class GL_Texture;

class GL_Material
{
	friend class GL_ResourceManager;
private:
	GL_Material();

public:
	~GL_Material();

	void Apply();

	inline void diffuse_color (float r, float g, float b, float a) {diffuse_color_.set(r, g, b, a);}
	inline void specular_color(float r, float g, float b, float a) {specular_color_.set(r, g, b, a);}
	inline void ambient_color (float r, float g, float b, float a) {ambient_color_.set(r, g, b, a);}
	inline void emission_color(float r, float g, float b, float a) {emission_color_.set(r, g, b, a);}
	inline void shininess(float s) {shininess_=s;}
	inline void shininess_strength(float s) { shininess_strength_ = s; }
	inline void ChangeAllAlphas(float a) { diffuse_color_[3] = specular_color_[3] = ambient_color_[3] = emission_color_[3] = a; }


	cml::vector4f ambient_color_f()  const { return (cml::vector4f)ambient_color_; }
	cml::vector4f specular_color_f() const { return (cml::vector4f)specular_color_; }
	cml::vector4f diffuse_color_f()  const { return (cml::vector4f)diffuse_color_; }
	float shininess_f() const { return (float)shininess_; }

	void AddTexture(GL_Texture *t) { textures_.push_back(t); }
	void RemoveTexture(GL_Texture *t);

	int CountTextures() const { return (int)textures_.size(); }
	GL_Texture* texture(int i) const { return textures_[i]; };

	bool flag_use_vertex_color() const { return flag_use_vertex_color_; }
	bool flag_texture() const { return flag_texture_; }
	bool flag_shading() const { return flag_shading_; }
	bool flag_flat_shading() const { return flag_flat_shading_; }
	bool flag_transparency() const { return flag_transparency_; }

	void flag_use_vertex_color(bool f) { flag_use_vertex_color_ = f; }
	void flag_texture(bool f) { flag_texture_ = f; }
	void flag_shading(bool f) { flag_shading_ = f; }
	void flag_flat_shading(bool f) { flag_flat_shading_ = f; }
	void flag_transparency(bool f) { flag_transparency_ = f; }
protected:

	bool flag_use_vertex_color_;
	bool flag_texture_;
	bool flag_shading_;
	bool flag_flat_shading_;
	bool flag_transparency_;

	cml::vector4f diffuse_color_;
	cml::vector4f specular_color_;
	cml::vector4f ambient_color_;
	cml::vector4f emission_color_;
	float shininess_;
	float shininess_strength_;

	std::vector<GL_Texture *> textures_;
};

};