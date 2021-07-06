


#include "BaseLib/BaseLib.h"
#include "BaseLib/GL4U/GL_Renderer.h"
#include "BaseLib/GL4U/GL_ShaderProgram.h"
#include "BaseLib/GL4U/GL_VBOVAO.h"
#include "BaseLib/GL4U/GL_RenderableObj.h"
#include "BaseLib/GL4U/GL_ResourceManager.h"
#include "BaseLib/GL4U/GL_Mesh.h"
#include "BaseLib/GL4U/GL_SceneGraph.h"
#include "BaseLib/Geometry/Mesh.h"
#include "BaseLib/Geometry/PrimitiveShape.h"
#include "BaseLib/CmlExt/CmlExt.h"
#include <assert.h>

namespace mg
{

GL_Renderer::GL_Renderer(int x, int y, int w, int h)
{
	// Default Shader
	{
		default_shader_ = std::move( std::unique_ptr<GL_ShaderProgram>(new GL_ShaderProgram) );
		default_shader_->CreateDefaultProgram();
		UseDefaultShader();


		picking_shader_ = std::move( std::unique_ptr<GL_ShaderProgram>(new GL_ShaderProgram) );
		picking_shader_->CreateFromFiles(std::string(GetBaseLibResourcePath())+"shaders/picking_vshader.glsl",
			std::string(GetBaseLibResourcePath())+"shaders/picking_fshader.glsl");
	}


	// Default Materal
	default_material_ = nullptr;
	CreateDefaultMateiral();

	// Camera
	InitDefaultCamera();
	UseDefaultCamera();
	view_mat_.identity();
	model_mat_.identity();

	// ShadowMap
	flag_shadowmap_ = false;
	shadow_depth_texture_id_ = -1;
	shadow_framebuffer_id_ = -1;

	clear_color_.set(1.f, 1.f, 1.f, 1.f);
	//InitShadowMap();


	flag_highlight_mode_ = false;
	flag_texture_ = true;
}



GL_Renderer::~GL_Renderer()
{
	Clear();
}

void
GL_Renderer::Clear()
{
	glUseProgram(0);
	ClearShadowMap();
}

void
GL_Renderer::ClearGLBuffers()
{
	glEnable(GL_CULL_FACE);
	glEnable(GL_DEPTH_TEST);

	glClearColor(clear_colorf()[0],
		clear_colorf()[1],
		clear_colorf()[2],
		clear_colorf()[3]
	);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void
GL_Renderer::SetViewport(int x, int y, int w, int h)
{
	if ( h==0 ) return;
	viewport_ = cml::vector4i(x, y, w, h);
	glViewport(x, y, w, h);
	camera_->setAspectRatio((double)w/h);
	UpdateProjectionViewMatrix();
}

////////////////////////////////////////////////////////////////////////
// Shader

void
GL_Renderer::UseDefaultShader()
{
	if ( shader_ != default_shader_.get() )
		UseShader(default_shader_.get());
}

void
GL_Renderer::UseShader(GL_ShaderProgram* s)
{
	if ( s==nullptr ) return;
	if ( shader_== s  ) return;
	shader_ = s;
	shader_->UseProgram();
}


///////////////////////////////////////
// Shader : Lighting

void 
GL_Renderer::SetAmbientLightIntensity(float l)
{
	if ( shader_==nullptr ) return;
	shader_->SetAmbientLightIntensity(l);
}

void 
GL_Renderer::EnableLight(int light_id, bool f)
{
	if ( shader_==nullptr ) return;
	if ( lights_.find(light_id) == lights_.end() )
		lights_[light_id] = GL_Light();
	shader_->EnableLight(light_id, f);
}


void 
GL_Renderer::SetLight(int light_id, const GL_Light &l)
{
	lights_[light_id] = l;

	UpdateShaderLight(light_id, false);
}

void 
GL_Renderer::SetLight(int light_id, int type, cml::vector3f dir, cml::vector3f position, cml::vector4f intensity, float cos_cutoff)
{
	GL_Light l;
	l.type = type;
	l.dir = dir;
	l.position = position;
	l.intensity = intensity;
	l.cos_cutoff = cos_cutoff;

	lights_[light_id] = l;
	UpdateShaderLight(light_id, false);
}

void 
GL_Renderer::SetLightPosition(int light_id, cml::vector3f position)
{
	lights_[light_id].position = position;
	UpdateShaderLight(light_id, false);
}

void 
GL_Renderer::SetLightDirection(int light_id, cml::vector3f dir)
{
	lights_[light_id].dir = dir;
	UpdateShaderLight(light_id, false);
}

void 
GL_Renderer::SetLightPosDir(int light_id, cml::vector3f position, cml::vector3f dir)
{
	lights_[light_id].position = position;
	lights_[light_id].dir = dir;
	UpdateShaderLight(light_id, false);
}

void
GL_Renderer::ApplyModelViewMatrixToLight(int light_id)
{
	UpdateShaderLight(light_id, true);
}

void 
GL_Renderer::DisableAllLight()
{
	if ( shader_==nullptr ) return;
	shader_->DisableAllLight();
}

void
GL_Renderer::UpdateShaderLight(int light_id, bool flag_applying_modelview_matrix)
{
	if ( shader_==nullptr ) return;

	GL_Light l = lights_[light_id];

	if ( flag_applying_modelview_matrix )
	{
		l.position = (view_mat_ * model_mat_ * cml::vector4d(l.position, 1)).subvector(3);
		l.dir = (view_mat_ * model_mat_ * cml::vector4d(l.dir, 0)).subvector(3);
	}

	shader_->SetLight(light_id, l);
}


void 
GL_Renderer::EnableHighlightMode(bool f)
{
	flag_highlight_mode_ = f;
	shader_->SetUniform<bool>("flag_highlight", f);
}


void 
GL_Renderer::EnableTexture(bool f)
{
	flag_texture_ = f;
	shader_->EnableTexture(f);
}


////////////////////////////////////////////////////////////////////////
// Camera

void
GL_Renderer::InitDefaultCamera()
{
	default_camera_.setFov(cml::rad(45.f));
	default_camera_.setNearFar(1.0f, 10000.0f);
	default_camera_.setAspectRatio((double)viewport_[2]/viewport_[3]);

	default_camera_.setTranslation({0.f, 150.f, 500.0f});
	default_camera_.setRotation({M_PI/30, M_PI, 0.0f});
}


void
GL_Renderer::UseDefaultCamera()
{
	if ( camera_ != &default_camera_ )
		UseCamera(&default_camera_);
}

void
GL_Renderer::UseCamera(cml::Camera *c)
{
	camera_ = c;
	UpdateProjectionViewMatrix();
}

void
GL_Renderer::UpdateProjectionViewMatrix()
{
	SetProjectionMatrix(camera_->GetGLProjectionMatrix());
	SetViewMatrix(camera_->GetGLViewMatrix());
}


void 
GL_Renderer::SetModelIdentity()
{
	cml::matrix44d i;
	i.identity();
	SetModelMatrix(i);
}



void 
GL_Renderer::PushModelMatrix()
{
	model_mat_stack_.push(model_mat_);
}

void
GL_Renderer::MultiModelMatrix(const cml::matrix44d &m)
{
	SetModelMatrix(model_mat_*m);
}

void 
GL_Renderer::PopModelMatrix()
{
	SetModelMatrix(model_mat_stack_.top());
	model_mat_stack_.pop();
}



void 
GL_Renderer::SetProjectionMatrix(const cml::matrix44d &m)
{
	shader_->SetProjectionMatrix(((cml::matrix44f_c)m).data());
}

void 
GL_Renderer::SetViewMatrix(const cml::matrix44d &m)
{
	view_mat_ = m;
	shader_->SetViewMatrix(((cml::matrix44f_c)m).data());
}


void 
GL_Renderer::SetModelMatrix(const cml::matrix44d &m)
{
	model_mat_ = m;
	shader_->SetModelMatrix(((cml::matrix44f_c)m).data());
}



////////////////////////////////////////////////////////////////////////
// Drawings

void 
GL_Renderer::DrawRenderableObjGroup(const std::string &renderable_obj_group_name) 
{
	DrawRenderableObjGroup( GL_ResourceManager::singleton()->GetRenderableObjGroup(renderable_obj_group_name) );
}

void 
GL_Renderer::DrawRenderableObjGroup(const GL_RenderableObjGroup *obj_group) 
{
	for ( unsigned int i=0; i<obj_group->renderable_objs_.size(); i++ )
	{
		PushModelMatrix();
		MultiModelMatrix(obj_group->global_t_of_objs_[i]);
		Draw(obj_group->renderable_objs_[i]);
		PopModelMatrix();
	}
}

void 
GL_Renderer::Draw(const std::string &renderable_obj_name) 
{
	Draw( GL_ResourceManager::singleton()->GetRenderableObj(renderable_obj_name) );
}

void
GL_Renderer::Draw(const GL_RenderableObj *obj) 
{
	assert(obj!=nullptr);

	// Skinning
	shader_->EnableSkinning(obj->flag_skinning());
	
	if ( obj->flag_skinning() )
	{
		for ( const auto &bone_id : obj->using_bone_matrix_id_set() )
		{
			cml::matrix44f m_and_offset = obj->GetBoneMatrix(bone_id) * cml::inverse(obj->GetBoneOffsetMatrix(bone_id));
			shader_->SetBoneMatrix(bone_id, ((cml::matrix44f_c)m_and_offset).data());

			//std::cout << "B: " << bone_id << " " << obj->GetBoneMatrix(0) <<std::endl;
		}
	}

	// Material
	if ( obj->flag_material() && obj->material() )
	{
		ApplyMaterial(obj->material());
	}
	else
	{
		ApplyMaterial(default_material_);
	}


	// Draw
	if ( obj->mesh() != nullptr )
	{
		obj->mesh()->Draw();
	}
	else DrawVAO(obj->vao());

	if ( obj->flag_skinning() ) shader_->EnableSkinning(false);

}



void
GL_Renderer::Draw(const GL_SceneNode *s_node)
{
	PushModelMatrix();
	MultiModelMatrix(s_node->transf().GetMat44());
	for ( int i=0; i<s_node->num_renderable_objs(); i++ )
	{
		Draw(s_node->renderable_obj(i));
	}
	for ( int i=0; i<s_node->num_children(); i++ )
	{
		Draw(s_node->child(i));
	}
	PopModelMatrix();
}


void
GL_Renderer::Draw(GL_Mesh *mesh)
{
	assert(obj != nullptr);

	ApplyMaterial(default_material_);

	// Draw
	if ( mesh->vao() == nullptr ) mesh->BuildVAO();
	mesh->Draw();
}





void
GL_Renderer::CreateDefaultMateiral()
{
	if (default_material_ == nullptr)
	{
		std::string name = GL_ResourceManager::singleton()->GenUniqueName("__default_material__");
		default_material_ = GL_ResourceManager::singleton()->CreateMaterial(name);
	}
}

void 
GL_Renderer::ApplyMaterial(const GL_Material *mat)
{
	shader_->SetMaterial(*mat, flag_texture_);

	if ( flag_texture_ && mat->flag_texture() )
	{
		if ( mat->CountTextures() > 0 )
		{
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

			if ( mat->texture(0)->levels() > 1 )
				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
			else
				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, mat->texture(0)->gl_texture_wrap_s());
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, mat->texture(0)->gl_texture_wrap_t());
		}
	}

	if ( mat->flag_transparency() )
	{
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	}
	else
	{
		glDisable(GL_BLEND);
	}
}

void 
GL_Renderer::DrawVAO(const GL_VAO *vao)
{
	vao->Draw();
}

void 
GL_Renderer::DrawSphere()
{
	Draw("primi_sphere"); 
}

void 
GL_Renderer::DrawSphere(double radius, const cml::vector3d &center_p) 
{
	PushModelMatrix();
	MultiModelMatrix(cml::MatrixTranslation(center_p));
	MultiModelMatrix(cml::MatrixUniformScaling(radius));
	Draw("primi_sphere"); 
	PopModelMatrix();
}

void GL_Renderer::DrawCapsule() 
{ 
	Draw("primi_capsule"); 
}

void GL_Renderer::DrawCapsule(double cylinder_len, double radius) 
{ 
	DrawHemisphere(radius, cml::vector3d(0.0, cylinder_len/2.0, 0.0));
	DrawCylinder(cylinder_len, radius);
	PushModelMatrix();
	MultiModelMatrix(cml::MatrixRotationEuler(M_PI, 0.0, 0.0));
	DrawHemisphere(radius, cml::vector3d(0.0, cylinder_len/2.0, 0.0));
	PopModelMatrix();
}


void GL_Renderer::DrawHemisphere(double radius, const cml::vector3d &center_p) 
{ 
	PushModelMatrix();
	MultiModelMatrix(cml::MatrixTranslation(center_p));
	MultiModelMatrix(cml::MatrixUniformScaling(radius));
	Draw("primi_hemisphere"); 
	PopModelMatrix();
}

void GL_Renderer::DrawHemisphere() 
{ 
	Draw("primi_hemisphere"); 
}

void GL_Renderer::DrawHead() 
{ 
	Draw("primi_head"); 
}

void GL_Renderer::DrawCylinder() 
{ 
	Draw("primi_cylinder"); 
}

void GL_Renderer::DrawCylinder(double cylinder_len, double radius) 
{
	PushModelMatrix();
	MultiModelMatrix(cml::MatrixScaling(cml::vector3d(radius, cylinder_len, radius)));
	Draw("primi_cylinder"); 
	PopModelMatrix();
}

void GL_Renderer::DrawCylinder(const cml::vector3d &bottom_p, const cml::vector3d &top_p, double radius) 
{
	cml::vector3d align_vec = top_p-bottom_p;
	cml::matrix44d align_rot;
	cml::matrix_rotation_vec_to_vec(align_rot, cml::y_axis_3D(), align_vec);

	PushModelMatrix();
	MultiModelMatrix(cml::MatrixTranslation(bottom_p));
	MultiModelMatrix(align_rot);
	MultiModelMatrix(cml::MatrixScaling(cml::vector3d(radius, align_vec.length(), radius)));
	MultiModelMatrix(cml::MatrixTranslation(0.0, 0.5, 0.0));
	Draw("primi_cylinder"); 
	PopModelMatrix();
}

void GL_Renderer::DrawOpenedCylinder() 
{ 
	Draw("primi_opened_cylinder"); 
}

void GL_Renderer::DrawBox(double width, double height, double depth) 
{ 
	PushModelMatrix();
	MultiModelMatrix(cml::MatrixScaling(width, height, depth));
	Draw("primi_box"); 
	PopModelMatrix();
}

void GL_Renderer::DrawQuad(double width, double depth) 
{ 
	PushModelMatrix();
	MultiModelMatrix(cml::MatrixScaling(width, 1., depth));
	Draw("primi_quad"); 
	PopModelMatrix();
}

void GL_Renderer::DrawQuadXY(double width, double height) 
{ 
	PushModelMatrix();
	MultiModelMatrix(cml::MatrixRotationEuler(cml::pi()/2., 0., 0.));
	MultiModelMatrix(cml::MatrixScaling(width, 1., height));
	Draw("primi_quad"); 
	PopModelMatrix();
}

void
GL_Renderer::Draw(const mg::PrimitiveShape *p) 
{
	PushModelMatrix();
	MultiModelMatrix(cml::MatrixTranslation((p->global_translation())));
	MultiModelMatrix(cml::MatrixRotationQuaternion((p->global_rotation())));
	switch (p->type())
	{
	case mg::PrimitiveShape::BOX:
	{
		mg::PrimitiveBox* box = (mg::PrimitiveBox*)p;
		DrawBox(box->width(), box->height(), box->depth());
		break;
	}

	case mg::PrimitiveShape::SPHERE:
	{
		mg::PrimitiveSphere* sphere = (mg::PrimitiveSphere*)p;
		DrawSphere(sphere->radius());
		break;
	}

	case mg::PrimitiveShape::CYLINDER:
	{
		mg::PrimitiveCylinder* cyl = (mg::PrimitiveCylinder*)p;
		DrawCylinder(cyl->height(), cyl->radius());
		break;
	}

	case mg::PrimitiveShape::CAPSULE:
	{
		mg::PrimitiveCapsule* cap = (mg::PrimitiveCapsule*)p;
		DrawCapsule(cap->cylinder_height(), cap->radius());
		break;
	}

	default:
		break;
	}
	PopModelMatrix();
}

void
GL_Renderer::Draw(const mg::PrimitiveComposition *p)
{
	PushModelMatrix();
	MultiModelMatrix(cml::MatrixTranslation((p->global_translation())));
	MultiModelMatrix(cml::MatrixRotationQuaternion((p->global_rotation())));

	for (int i = p->CountPrimitives() - 1; i >= 0; i--)
	{
		Draw(p->primitive(i));
	}
	PopModelMatrix();

}









/////////////////////////////
// Indexed Color

static int g_last_used_color_index;
static const int g_indexed_color_num = 42;
static const float g_indexed_color_set[g_indexed_color_num][3] = { 
			{0.7f, 1.0f, 1.0f},
			{1.0f, 0.7f, 0.7f},
			{0.7f, 1.0f, 0.7f},
			{0.7f, 0.7f, 1.0f},
			{0/255.0f,	204/255.0f,	255/255.0f},	
			{204/255.0f,	255/255.0f,	204/255.0f},
			{255/255.0f,	255/255.0f,	153/255.0f},
			{153/255.0f,	204/255.0f,	255/255.0f},
			{255/255.0f,	153/255.0f,	204/255.0f},
			{204/255.0f,	153/255.0f,	255/255.0f},	// 10
			{255/255.0f,	204/255.0f,	153/255.0f},
			{51/255.0f,	102/255.0f,	255/255.0f},
			{51/255.0f,	204/255.0f,	204/255.0f},
			{153/255.0f,	204/255.0f,	0/255.0f},
			{255/255.0f,	204/255.0f,	0/255.0f},
			{255/255.0f,	153/255.0f,	0/255.0f},
			{255/255.0f,	102/255.0f,	0/255.0f},
			{102/255.0f,	102/255.0f,	153/255.0f},
			{150/255.0f,	150/255.0f,	150/255.0f},
			{0/255.0f,	51/255.0f,	102/255.0f},	// 20
			{51/255.0f,	153/255.0f,	102/255.0f},
			{0/255.0f,	51/255.0f,	0/255.0f},
			{51/255.0f,	51/255.0f,	0/255.0f},
			{153/255.0f,	51/255.0f,	0/255.0f},
			{51/255.0f,	51/255.0f,	153/255.0f},
			{51/255.0f,	51/255.0f,	51/255.0f},
			{128/255.0f,	0/255.0f,	0/255.0f},
			{0/255.0f,	128/255.0f,	0/255.0f},
			{0/255.0f,	0/255.0f,	128/255.0f},
			{128/255.0f,	128/255.0f,	0/255.0f},	// 30
			{128/255.0f,	0/255.0f,	128/255.0f},
			{0/255.0f,	128/255.0f,	128/255.0f},
			{192/255.0f,	192/255.0f,	192/255.0f},
			{128/255.0f,	128/255.0f,	128/255.0f},
			{153/255.0f,	153/255.0f,	255/255.0f},
			{153/255.0f,	51/255.0f,	102/255.0f},
			{255/255.0f,	255/255.0f,	204/255.0f},
			{204/255.0f,	255/255.0f,	255/255.0f},
			{102/255.0f,	0/255.0f,	102/255.0f},
			{255/255.0f,	128/255.0f,	128/255.0f},	// 40
			{0/255.0f,	102/255.0f,	204/255.0f},
			{204/255.0f,	204/255.0f,	255/255.0f}
			};


cml::vector3f 
GL_Renderer::GetIndexedColor(int index)
{
	index = index % 42;
	
	cml::vector3f c;
	c[0] = g_indexed_color_set[index][0];
	c[1] = g_indexed_color_set[index][1];
	c[2] = g_indexed_color_set[index][2];

	return c;
}

void 
GL_Renderer::SetColor(int i, float a)
{
	int r = i % g_indexed_color_num;
	SetColor(g_indexed_color_set[r][0], g_indexed_color_set[r][1], g_indexed_color_set[r][2], a);
	g_last_used_color_index = r;
}

void
GL_Renderer::SetColor(float r, float g, float b, float a)
{
	//shader_->SetColor(r, g, b, a);
	default_material_->diffuse_color(r*0.8f, g*0.8f, b*0.8f, a);
	default_material_->ambient_color(r*0.2f, g*0.2f, b*0.2f, a);
}

void
GL_Renderer::SetColor(const cml::vector4f &c)
{
	SetColor(c[0], c[1], c[2], c[3]);
}

void
GL_Renderer::SetAlpha(float a)
{
	default_material_->ChangeAllAlphas(a);
}

void
GL_Renderer::EnableTransparency(bool f)
{
	default_material_->flag_transparency(f);
}

int
GL_Renderer::GetRecentlyUsedColorIndex()
{
	return g_last_used_color_index;
}



/////////////////////////////////////////////////////////////////////
// Shadow Map Render
/// See  http://fabiensanglard.net/shadowmapping/index.php
void
GL_Renderer::InitShadowMap()
{
	shadow_map_width_ = 1024;
	shadow_map_height_ = 1024;


	GLenum FBOstatus;

	// Try to use a texture depth component
	glGenTextures(1, &shadow_depth_texture_id_);
	glBindTexture(GL_TEXTURE_2D, shadow_depth_texture_id_);

	// GL_LINEAR does not make sense for depth texture. However, next tutorial shows usage of GL_LINEAR and PCF
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

	// Remove artefact on the edges of the shadowmap
	glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP );
	glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP );

	//glTexParameterfv( GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, borderColor );



	// No need to force GL_DEPTH_COMPONENT24, drivers usually give you the max precision if available 
	glTexImage2D( GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, shadow_map_width_, shadow_map_height_, 0, GL_DEPTH_COMPONENT, GL_UNSIGNED_BYTE, 0);
	
	// Unbind
	glBindTexture(GL_TEXTURE_2D, 0);

	// create a framebuffer object
	glGenFramebuffersEXT(1, &shadow_framebuffer_id_);
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, shadow_framebuffer_id_);

	// Instruct openGL that we won't bind a color texture with the currently binded FBO
	glDrawBuffer(GL_NONE);
	glReadBuffer(GL_NONE);

	// attach the texture to FBO depth attachment point
	glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT,GL_TEXTURE_2D, shadow_depth_texture_id_, 0);

	// check FBO status
	FBOstatus = glCheckFramebufferStatusEXT(GL_FRAMEBUFFER_EXT);
	if(FBOstatus != GL_FRAMEBUFFER_COMPLETE_EXT)
		printf("GL_FRAMEBUFFER_COMPLETE_EXT failed, CANNOT use FBO\n");

	// switch back to window-system-provided framebuffer
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);
}

void
GL_Renderer::ClearShadowMap()
{
	if ( shadow_depth_texture_id_ != -1 )
	{
		glDeleteTextures(1, &shadow_depth_texture_id_);
	}
	shadow_depth_texture_id_ = -1;

	if ( shadow_framebuffer_id_ != -1 )
	{
		glDeleteFramebuffers(1, &shadow_framebuffer_id_);
	}
	shadow_framebuffer_id_ = -1;
}


cml::Camera *g_shadowmap_tmp_camera_ptr = nullptr;
GL_ShaderProgram *g_shadowmap_tmp_shader_ptr = nullptr;
cml::vector4i g_shadowmap_tmp_viewport;

void
GL_Renderer::EnableShadowMap(bool f)
{
	if ( flag_shadowmap_ == f ) return;
	
	if ( f )
	{
		InitShadowMap();
		shader_->EnableShadowMap(f);
	}
	else
	{
		ClearShadowMap();
		shader_->EnableShadowMap(f);
	}

	flag_shadowmap_ = f;
}

void 
GL_Renderer::BeginShadowMapRender()
{
	//First step: Render from the light POV to a FBO, story depth values only
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, shadow_framebuffer_id_);	//Rendering offscreen

	//Using the fixed pipeline to render to the depthbuffer
	//glUseProgramObjectARB(0);
	g_shadowmap_tmp_shader_ptr = shader();
	UseDefaultShader();

	// Set the camera watching from the light POV
	//setupMatrices(p_light[0],p_light[1],p_light[2],l_light[0],l_light[1],l_light[2]);
	g_shadowmap_tmp_camera_ptr = camera();
	UseCamera(&shadowmap_light_camera_);

	g_shadowmap_tmp_viewport = viewport_;
	// In the case we render the shadowmap to a higher resolution, the viewport must be modified accordingly.
	glViewport(0, 0, shadow_map_width_, shadow_map_height_);

	// Clear previous frame values
	glClear(GL_DEPTH_BUFFER_BIT);

	//Disable color rendering, we only want to write to the Z-Buffer
	glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE); 

	
	
	PushModelMatrix();
	//shader_->SetShadowmapLightProjectionViewMatrix(((cml::matrix44f_c)shadowmap_light_camera_.GetGLProjectionViewMatrix()).data());

	// Culling switching, rendering only backface, this is done to avoid self-shadowing
	glCullFace(GL_FRONT);
}

void 
GL_Renderer::EndShadowMapRender()
{
	//Using the original shadow shader
	//glUseProgramObjectARB(shadowShaderId);
	UseShader(g_shadowmap_tmp_shader_ptr);

	////Save modelview/projection matrice into texture7, also add a biais
	// This is matrix transform every coordinate x,y,z
	// x = x* 0.5 + 0.5 
	// y = y* 0.5 + 0.5 
	// z = z* 0.5 + 0.5 
	// Moving from unit cube [-1,1] to [0,1]  
	cml::matrix44d bias = cml::MatrixTranslation(0.5, 0.5, 0.5) * cml::MatrixScaling(0.5, 0.5, 0.5);
	cml::matrix44d bias_light_m = bias * shadowmap_light_camera_.GetGLProjectionViewMatrix();
	shader_->SetShadowmapLightProjectionViewMatrix(((cml::matrix44f_c)bias_light_m).data());
	// setTextureMatrix();
	shader_->BindShadowmapTextureId(shadow_depth_texture_id_);


	// Now rendering from the camera POV, using the FBO to generate shadows
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT,0);

	SetViewport(g_shadowmap_tmp_viewport[0], g_shadowmap_tmp_viewport[1], g_shadowmap_tmp_viewport[2], g_shadowmap_tmp_viewport[3]);

	//Enabling color write (previously disabled for light POV z-buffer rendering)
	glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE); 

	// Clear previous frame values
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	



	// Set back the view camera
	//setupMatrices(p_camera[0],p_camera[1],p_camera[2],l_camera[0],l_camera[1],l_camera[2]);
	UseCamera(g_shadowmap_tmp_camera_ptr);
	PopModelMatrix();
	glCullFace(GL_BACK);
}



///////////////////////////////////////
// Picking
void 
GL_Renderer::BeginPickingRender()
{

	shader_tmp_ = shader_;
	UseShader(picking_shader_.get());

	
}

void 
GL_Renderer::SetPickName(unsigned int name)
{
	// Convert "name", the integer mesh ID, into an RGB color
	unsigned int r = (name & 0x000000FF) >>  0;
	unsigned int g = (name & 0x0000FF00) >>  8;
	unsigned int b = (name & 0x00FF0000) >> 16;


	picking_shader_->SetUniform<cml::vector3f>("picking_name", {r/255.f, g/255.f, b/255.f});
}

unsigned int 
GL_Renderer::Pick(int win_x, int win_y)
{
	GLint viewport[4];
	glGetIntegerv(GL_VIEWPORT,viewport);

	unsigned int picked_name = 0;	// when nothing is picked.

	unsigned char data[4];

	glFlush();
	glFinish();
	glReadPixels(win_x, viewport[ 3 ] - win_y, 1 , 1, GL_RGBA, GL_UNSIGNED_BYTE, data);

	picked_name = 
		data[0] + 
		data[1] * 256 +
		data[2] * 256*256;

	return picked_name;
}

bool 
GL_Renderer::PickPositionOnRenderedSurface(int win_x, int win_y, cml::vector3d &out)
{
	glFlush();
	glFinish();
	GLint viewport[4];
	glGetIntegerv(GL_VIEWPORT,viewport);


	// Get the Z coordinate for the pixel location
	GLuint lDepth;
	glReadPixels( win_x, ( viewport[ 3 ] - win_y ), 
		1, 1, GL_DEPTH_COMPONENT, GL_UNSIGNED_INT, 
		&lDepth );

	if ( lDepth == UINT_MAX ) return false;

	int zdepth;
	double lDistTermInv;
	glGetIntegerv( GL_DEPTH_BITS, &zdepth );
	switch ( zdepth )
	{
	case 16 :
		lDistTermInv = 1.5259018967e-5; // 65535
		break;
	default :
		lDistTermInv = 2.32830643708e-10; // 4294967295
		break;
	}
	lDistTermInv = 1./pow(2, 32);

	double lDistance = lDepth * lDistTermInv;

	double *mv = camera_->GetGLViewMatrix().data();
	double *p = camera_->GetGLProjectionMatrix().data();

	out = UnProjectWinP( win_x, win_y, lDistance, *camera_);

	return true;
}




void 
GL_Renderer::EndPickingRender()
{
	UseShader(shader_tmp_);

	glClearColor(clear_colorf()[0],
		clear_colorf()[1],
		clear_colorf()[2],
		clear_colorf()[3]
	);
}







/////////////////////////////////////////////////////////////////////
// OpenGL 1.x Style Drawing (Inefficient)
static GL_VBOGroup *g_gl1_vbo_group = nullptr;
static GL_Mesh *g_gl1_mesh = nullptr;
void 
GL_Renderer::glBegin(GLenum drawing_mode)
{
	if ( g_gl1_vbo_group != nullptr )
		delete g_gl1_vbo_group;

	g_gl1_vbo_group = new GL_VBOGroup;
	g_gl1_vbo_group->drawing_mode(drawing_mode);

	g_gl1_vbo_group->BeginVertex();
}

void 
GL_Renderer::glNormal(cml::vector3f n)
{
	g_gl1_vbo_group->glNormal(n);
}

void 
GL_Renderer::glUV(cml::vector2f uv)
{
	g_gl1_vbo_group->glUV(uv);
}

void 
GL_Renderer::glColor(cml::vector4f c)
{
	g_gl1_vbo_group->glColor(c);
}

void 
GL_Renderer::glVertex(cml::vector3f v)
{
	g_gl1_vbo_group->glVertex(v);
}

void 
GL_Renderer::glEnd()
{
	g_gl1_vbo_group->EndVertex();

	std::unique_ptr<GL_VAO> vao(new GL_VAO(g_gl1_vbo_group));
	std::unique_ptr<GL_RenderableObj> obj(new GL_RenderableObj);
	obj->Initialize(vao.get());
	Draw(obj.get());

	delete g_gl1_vbo_group;
	g_gl1_vbo_group = nullptr;
}


static GL_VBOGroup *g_mesh_vbo_group = nullptr;
void 
GL_Renderer::DrawMesh(mg::Mesh *mesh)
{
	if ( g_mesh_vbo_group != nullptr )
		delete g_mesh_vbo_group;

	g_mesh_vbo_group = new GL_VBOGroup;
	g_mesh_vbo_group->SetByMesh(*mesh);

	std::unique_ptr<GL_VAO> vao(new GL_VAO(g_mesh_vbo_group));
	std::unique_ptr<GL_RenderableObj> obj(new GL_RenderableObj);

	obj->Initialize(vao.get());
	Draw(obj.get()); 

	delete g_mesh_vbo_group;
	g_mesh_vbo_group = nullptr;
}

void
GL_Renderer::DrawString(GL_Font* font, std::string str)
{
	shader_->EnableTexture(true);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	font->DrawString(str);
	EnableTexture(flag_texture_);
}

void
GL_Renderer::DrawString(std::string str)
{
	shader_->EnableTexture(true);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	GL_ResourceManager::singleton()->GetFont()->DrawString(str);
	EnableTexture(flag_texture_);
}

};
