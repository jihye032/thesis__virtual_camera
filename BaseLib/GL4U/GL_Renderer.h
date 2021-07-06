
#pragma once
#include "BaseLib/CmlExt/CmlExt.h"
#include "BaseLib/CmlExt/CmlCamera.h"
#include "BaseLib/GL4U/GL_ShaderProgram.h"
#include "GL/glew.h"
#include <map>
#include <string>
#include <stack>



namespace mg
{

class GL_RenderableObj;
class GL_RenderableObjGroup;
class GL_Material;
class GL_Texture;
class GL_VAO;
class GL_Mesh;
class PrimitiveShape;
class PrimitiveComposition;
class Mesh;
class GL_SceneNode;
class GL_Font;

/**
This class includes algorithms and unitility function to render a instance of GL_RenderableObj.
The rendering processes are (1)reading the rendering information in the GL_RenderableObj instance,
(2)setting default_shader_ (or shader_) parameters and (3)calling profer glew functions.

And it provides unitility functions that makes easier to set and manage the shader parameters 
and glew paramemters. (e.g. managing matrix stack - Push/PopModelViewMatrix)
*/

class GL_Renderer
{
public:
	/*
	x, y, w, are h are the viewport parameters.
	*/
	GL_Renderer(int x=0, int y=0, int w=0, int h=0);	
	virtual ~GL_Renderer();
	void Clear();

	void ClearGLBuffers();

	void SetViewport(int x, int y, int w, int h);
	void clear_color(cml::vector4f c) { clear_color_ = c; }
	cml::vector4f clear_color() const { return clear_color_; }
	const float* clear_colorf() const { return clear_color_.data(); }

	void Draw(const std::string &renderable_obj_name);
	void Draw(const GL_RenderableObj *obj);
	void Draw(GL_Mesh *mesh);
	void DrawRenderableObjGroup(const std::string &renderable_obj_group_name);
	void DrawRenderableObjGroup(const GL_RenderableObjGroup *group);
	void DrawSphere();
	void DrawSphere(double radius, const cml::vector3d &center_p=cml::vector3d(0, 0, 0));
	void DrawCapsule();
	void DrawCapsule(double cylinder_len, double radius);
	void DrawHemisphere();
	void DrawHemisphere(double radius, const cml::vector3d &center_p=cml::vector3d(0, 0, 0));
	void DrawHead();
	void DrawCylinder();
	void DrawCylinder(double cylinder_len, double radius);
	void DrawCylinder(const cml::vector3d &bottom_p, const cml::vector3d &top_p, double radius=1.0);
	void DrawOpenedCylinder();
	void DrawBox(double width=1.0, double height=1.0, double depth=1.0);
	void DrawQuad(double width=1.0, double depth=1.0);
	void DrawQuadXY(double width=1.0, double height=1.0);
	void Draw(const PrimitiveShape* p);
	void Draw(const PrimitiveComposition* p);
	void Draw(const GL_SceneNode* s_node);

	//////////////////////////////////////////////////////////////////
	// Text
	void DrawString(GL_Font* font, std::string str);
	void DrawString(std::string str);

	

	void ApplyMaterial(const GL_Material *m=nullptr);
	void DrawVAO(const GL_VAO *vao);

	// Default Material Color
	void SetColor(int i, float a=1.0);
	// Default Material Color
	void SetColor(float r, float g, float b, float a=1.0);
	void SetColor(const cml::vector4f &c);
	// Default Material Color
	int  GetRecentlyUsedColorIndex();
	// Default Material Alpha
	void SetAlpha(float a);
	// Default Material Tramsparency
	void EnableTransparency(bool t);


	static cml::vector3f GetIndexedColor(int index);

	///////////////////////////////////////
	// Camera, Model, View, and Projection Matrices.
	void UseDefaultCamera();
	void UseCamera(cml::Camera *c);
	cml::Camera* camera() const { return camera_; }
	const cml::Camera* default_camera() const { return &default_camera_; }
	
	void PushModelMatrix();
	void SetViewMatrix(const cml::matrix44d &m);
	void SetModelMatrix(const cml::matrix44d &m);
	void SetModelIdentity();
	void MultiModelMatrix(const cml::matrix44d &m);
	void PopModelMatrix();

	/**
	This method updates the internal projection matrix and view matrix based on the current 
	camera parameters. cml::Camera has the aspect ratio parameter,
	which is changed when SetViewport() is called. So, UpdateProjectionViewMatrix() is
	called inside of SetViewport()- you don't need to call it after SetViewport().
	*/
	void UpdateProjectionViewMatrix();



	///////////////////////////////////////
	// Shader
	void UseShader(GL_ShaderProgram* s);
	void UseDefaultShader();
	GL_ShaderProgram* shader() const { return shader_; }


	///////////////////////////////////////
	// Shader : Lighting
	void SetAmbientLightIntensity(float l);
	void EnableLight(int light_id, bool f = true);
	void SetLight(int light_id, const GL_Light &l);
	void SetLight(int light_id, int type, cml::vector3f dir, cml::vector3f position = cml::vector3f(0.f, 0.f, 0.f), cml::vector4f intensity = cml::vector4f(1.f, 1.f, 1.f, 1.f), float cos_cutoff = 0.f);
	void SetLightPosition(int light_id, cml::vector3f position);
	void SetLightDirection(int light_id, cml::vector3f dir_from_src_to_target);
	void SetLightPosDir(int light_id, cml::vector3f position, cml::vector3f dir);
	void ApplyModelViewMatrixToLight(int light_id);
	void DisableAllLight();

	

	///////////////////////////////////////
	// Material
	GL_Material* default_material() const { return default_material_; }


	

	///////////////////////////////////////
	// Shawdow Map Rendering
	bool flag_shadowmap_;
	void EnableShadowMap(bool f);
	void BeginShadowMapRender();
	void EndShadowMapRender();
	cml::Camera& shadowmap_light_camera_editable() { return shadowmap_light_camera_; }
	const cml::Camera& shadowmap_light_camera() const { return shadowmap_light_camera_; }


	///////////////////////////////////////
	// Picking
	void BeginPickingRender();
	void EndPickingRender();
	/**
	@prama name must be greater than 0. Do NOT use 0.
	*/
	void SetPickName(unsigned int name);
	/**
	Pick method must be called between BeginPickingRender and EndPickingRender
	@return When nothing is picked the return value is 0.
	*/
	unsigned int Pick(int win_x, int win_y);

	/**
	Pick method must be called between BeginPickingRender and EndPickingRender
	@return When nothing is picked the return value is 0.
	*/
	bool PickPositionOnRenderedSurface(int win_x, int win_y, cml::vector3d &out);


	// (Inefficient)
	void DrawMesh(mg::Mesh *mesh);

	///////////////////////////////////////
	// OpenGL 1.x Style (Inefficient)
	void glBegin(GLenum drawing_mode);
	// OpenGL 1.x Style (Inefficient)
	void glNormal(cml::vector3f n);
	// OpenGL 1.x Style (Inefficient)
	void glUV(cml::vector2f uv);
	// OpenGL 1.x Style (Inefficient)
	void glColor(cml::vector4f c);
	// OpenGL 1.x Style (Inefficient)
	void glVertex(cml::vector3f v);
	// OpenGL 1.x Style (Inefficient)
	void glEnd();




	//////////////////////////////////////////
	// Rendering Mode
	void EnableHighlightMode(bool f=true);
	void DisableHighlightMode() { EnableHighlightMode(false); }
	void EnableTexture(bool f=true);
	void DisableTexture() { EnableTexture(false); }


protected:
	void InitShadowMap();
	void InitDefaultCamera();
	void CreateDefaultMateiral();
	void ClearShadowMap();

	void SetProjectionMatrix(const cml::matrix44d &m);
	void UpdateShaderLight(int light_id, bool flag_applying_modelview_matrix);

protected:
	cml::vector4f clear_color_;
	std::unique_ptr<GL_ShaderProgram> default_shader_;
	std::unique_ptr<GL_ShaderProgram> picking_shader_;
	GL_ShaderProgram* shader_;
	GL_ShaderProgram* shader_tmp_;	//

	cml::matrix44d view_mat_;
	cml::matrix44d model_mat_;
	std::stack<cml::matrix44d> model_mat_stack_;


	GL_Material *default_material_;

	///////////////////////////////////////
	// viewpor
	cml::vector4i viewport_;

	///////////////////////////////////////
	// Camera
	cml::Camera default_camera_;
	cml::Camera *camera_;

	///////////////////////////////////////
	// Shawdow Map Rendering
	int shadow_map_width_;
	int shadow_map_height_;
	GLuint shadow_depth_texture_id_;
	GLuint shadow_framebuffer_id_;
	cml::Camera shadowmap_light_camera_;

	///////////////////////////////////////
	// Lights
	std::map<int, GL_Light> lights_;

	//////////////////////////////////////////
	// Rendering Mode
	bool flag_highlight_mode_;
	bool flag_texture_;
};	


mg::GL_Renderer* GetDefaultRenderer();
};

