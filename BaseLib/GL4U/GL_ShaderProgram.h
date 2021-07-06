
#pragma once
#include "BaseLib/CmlExt/CmlExt.h"
#include "GL/glew.h"
#include <string>

namespace mg
{

class GL_Material;
class GL_Texture;

class GL_Light
{
public:
	enum { DIRECTIONAL=0, POINT=1, SPOT=2 };
	// 0: Directionl Light
	// 1: Point Light
	// 2: Spot Light
	int type;


	cml::vector3f dir;
	cml::vector3f position;
	cml::vector4f intensity; // I_l
	float cos_cutoff;	// Spot Light
};

class GL_ShaderProgram
{
public:
	enum { V_POSITION_LOC=0, V_COLOR_LOC, V_NORMAL_LOC, V_TEX_COORD_LOC, V_BONE_ID_LOC, V_BONE_WEIGHT_LOC };
	enum { NO_SHADING=0, PHONG_SHADING, FLAT_SHADING };

	GL_ShaderProgram();
	~GL_ShaderProgram();

	void CreateDefaultProgram();
	GLuint CreateFromFiles(const std::string &v_shader_file, const std::string &f_shader_file);
	GLuint CreateFromSource(const std::string &v_shader_src, const std::string &f_shader_src);
	void UseProgram();
	bool IsUsing() const;
	void Delete();

	// Set Default Vertex Attributes
	void SetDefaultVertexColor(double r, double g, double b, double a = 1.0);
	void SetDefaultVertexColor(const cml::vector4f &c);

	// Set Uniform Values
	void SetShadingMode(int m = PHONG_SHADING);
	
	// Set Uniform Values : Matricies
	void SetProjectionMatrix(cml::matrix44f_c &m44_c);
	void SetViewMatrix(cml::matrix44f_c &m44_c);
	void SetModelMatrix(cml::matrix44f_c &m44_c);
	void SetProjectionMatrix(const float *m44_c);
	void SetViewMatrix(const float *m44_c);
	void SetModelMatrix(const float *m44_c);

	// Set Uniform Values : Mateirals
	void SetMaterial(const GL_Material &m, bool flag_texture=true);
	void SetPhongMaterial(const cml::vector4f &k_a, const cml::vector4f &k_d, const cml::vector4f &k_s, const float shinness);
	void EnableTexture(bool f);
	void EnableVertexColor(bool f);
	void SetTexture(const GL_Texture &t);

	// Set Uniform Values : Lighting
	void SetAmbientLightIntensity(float l);
	void EnableLight(int light_id, bool f = true);
	void SetLight(int light_id, const GL_Light &l);
	void SetLight(int light_id, int type, cml::vector3f dir, cml::vector3f position = cml::vector3f(0.f, 0.f, 0.f), cml::vector4f intensity = cml::vector4f(1.f, 1.f, 1.f, 1.f), float cos_cutoff = 0.f);
	/*
	void SetLightPosition(int light_id, cml::vector3f position);
	void SetLightDirection(int light_id, cml::vector3f dir);
	void SetLightPosDir(int light_id, cml::vector3f position, cml::vector3f dir);
	*/
	void DisableAllLight();
	GL_Light GetLight(int light_id) const;

	// Set Uniform Values : Skinning 
	void EnableSkinning(bool f);
	void SetBoneMatrix(int bone_id, cml::matrix44f_c &m44_c);
	void SetBoneMatrix(int bone_id, float *m44_c);

	// Set Uniform Values : Shadow 
	void SetShadowmapLightProjectionViewMatrix(cml::matrix44f_c &m44_c);
	void SetShadowmapLightProjectionViewMatrix(const float *m44_c);
	void EnableShadowMap(bool f);
	void BindShadowmapTextureId(GLuint texture_id_);

	// Set Uniform Values
	bool IsSkinningOn();

	inline GLint GetViewMatrixLoc();
	inline GLint GetModelMatrixLoc();
	inline GLint GetProjectionMatrixLoc();

	inline GLuint program_handle() const { return program_handle_; }


	// General Get-Set Methods
	void SetUniformMat4(std::string var_name, cml::matrix44f_c &m);

	/**
	@param T can be bool, int, unsignd int, float, double, cml::vector3f, cml::vector3d, 
	                cml::vector4f, cml::vector4d, or cml::matrix44f_c.
	*/
	template<class T>
	void SetUniform(std::string var_name, T d);

	/**
	@param T can be int, unsignd int, float, double, cml::vector3f, cml::vector3d, 
	                cml::vector4f, cml::vector4d, or cml::matrix44f_c.
	*/
	template<typename T>
	void SetUniformV(std::string var_name, int size, T* d);

protected:
	bool ReadSourceFromFile(const std::string &in_filename, std::string &out_src);
	GLint GetBoneMatrixLoc(int bone_id);

protected:
	bool never_used_yet_;
	GLuint program_handle_;
	GLint projection_matrix_loc_;
	GLint view_matrix_loc_;
	GLint model_matrix_loc_;
	GLint bone_matrix_loc_;
	GLint tex7_shadowmap_loc_;
	GLint shadowmap_tex_bias_light_pov_matrix_loc_;

	int max_num_lights_;
};


};