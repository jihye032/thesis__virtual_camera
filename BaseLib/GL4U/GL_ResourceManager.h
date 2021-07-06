
#pragma once
#include "GL/glew.h"
#include <map>
#include <string>
#include <stack>


#include "BaseLib/GL4U/GL_VBOVAO.h"
#include "BaseLib/GL4U/GL_Material.h"
#include "BaseLib/GL4U/GL_Texture.h"
#include "BaseLib/GL4U/GL_RenderableObj.h"
#include "BaseLib/GL4U/GL_SceneGraph.h"
#include "BaseLib/GL4U/GL_Font.h"

namespace mg
{


class GL_ResourceManager
{
private:
	GL_ResourceManager();
	static GL_ResourceManager* singleton_;

	void CreateAndAddPrimitiveShapes();
	void CreateAndAddGroundObj();

public:
	static GL_ResourceManager* singleton();

	GL_VAO*					CreateVAO(std::string name, GL_VBOGroup *vbo_group);
	GL_VBOGroup*			CreateVBOGroup(std::string name="");
	GL_Material*			CreateMaterial(std::string name="");
	GL_Texture*				CreateTexture(std::string name = "");
	GL_Mesh*				CreateMesh(std::string name="", Mesh *mesh=nullptr);
	GL_RenderableObj*		CreateRenderableObj(std::string name, GL_Material *material = nullptr);
	GL_RenderableObj*		CreateRenderableObj(std::string name, GL_Mesh *vao, GL_Material *material = nullptr);
	GL_RenderableObj*		CreateRenderableObj(std::string name, GL_VAO *vao, GL_Material *material=nullptr);
	GL_RenderableObj*		CreateRenderableObj(std::string name, GL_VBOGroup *vbo_group, GL_Material *material=nullptr);
	GL_RenderableObjGroup*	CreateRenderableObjGroup(std::string name);
	GL_Font*				CreateFontTexture(std::string name, std::string font_ttf_file, int font_size=12);
	GL_Font*				CreateFontTexture(int font_size=12);


	void DeleteRenderableObj(GL_RenderableObj *p) {/*TODO*/};
	void DeleteVAO(GL_VAO *p) {/*TODO*/};
	void DeleteVBOGroup(GL_VBOGroup *p) {/*TODO*/};


	GL_VAO* GetVAO(std::string name) const;
	GL_VBOGroup* GetVBOGroup(std::string name) const;
	GL_Material* GetMaterial(std::string name) const;
	GL_Texture* GetTexture(std::string name) const;
	GL_RenderableObj* GetRenderableObj(std::string name) const;
	GL_RenderableObj* GetRenderableObj(std::string name);
	GL_RenderableObjGroup* GetRenderableObjGroup(std::string name) const;
	GL_RenderableObjGroup* GetRenderableObjGroup(std::string name);
	GL_Mesh* GetMesh(std::string name) const;
	GL_Font* GetFont(std::string name="default_FONT_12") const;

	// GL_Material* GetDefaultMaterial() const { return GetMaterial("mat_default"); }

	std::string GenUniqueName(std::string prefix="") const;


	//GL_RenderableObjGroup* ImportByAssimp(const std::string filename, std::string group_name = "");
	//GL_SceneGraph* ImportByAssimp(const std::string filename, std::string name="");
	void ImportByAssimp(GL_SceneNode* out_root_node, const std::string filename, std::string name="");

	const std::map< std::string, std::unique_ptr<GL_VAO> >& vao_set() { return vao_set_; }
	const std::map< std::string, std::unique_ptr<GL_Mesh> >& mesh_set() { return mesh_set_; }

private:
	std::map< std::string, std::unique_ptr<GL_Mesh> > mesh_set_;
	std::map< std::string, std::unique_ptr<GL_VAO> > vao_set_;
	std::map< std::string, std::unique_ptr<GL_VBOGroup> > vbo_group_set_;
	std::map< std::string, std::unique_ptr<GL_Material> > material_set_;
	std::map< std::string, std::unique_ptr<GL_Texture> > texture_set_;
	std::map< std::string, std::unique_ptr<GL_RenderableObj> > renderable_obj_set_;
	std::map< std::string, std::unique_ptr<GL_RenderableObjGroup> > renderable_obj_group_set_;
	std::map< std::string, std::unique_ptr<GL_Font> > font_set_;

};	





};

