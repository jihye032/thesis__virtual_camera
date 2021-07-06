
#pragma once

#include "GL/glew.h"
#include "BaseLib/CmlExt/CmlExt.h"
#include "BaseLib/CmlExt/CmlCamera.h"
#include "BaseLib/GL4U/GL_RenderableObj.h"
#include "BaseLib/Geometry/TRS_Transf.h"
#include <memory>
#include <vector>

namespace mg
{
class Mesh;

/**
GL_SceneNode has three main components, Transformation Info, Renderable Objects, and Childen nodes.
*/

class GL_SceneNode
{
	friend class GL_ResourceManager;
protected:
	GL_SceneNode();

public:
	//////////////////////////////////////////////////
	// Transformation.
	TRS_Transf& transf() { return transf_; }
	const TRS_Transf& transf() const { return transf_; }
	void transf(TRS_Transf t) {	transf_ = t; }


	///////////////////////////////////////////////////
	// Renderable Objects.
	void AddRenderableObj(mg::GL_RenderableObj* obj);
	void AddRenderableObj(std::string name);
	int num_renderable_objs() const { return (int)renderable_objs_.size(); }
	GL_RenderableObj* renderable_obj(int i) const { return renderable_objs_[i]; }
	const std::vector<GL_RenderableObj*>& renderable_objs() const { return renderable_objs_; }
	

	///////////////////////////////////////////////////
	// Children.
	GL_SceneNode* CreateChild();
	void RemoveChild(GL_SceneNode *c);
	int num_children() const { return (int)children_.size(); }
	GL_SceneNode* child(int i) const { return children_[i].get(); }

	/**
	Add the copies of n and n's all descenders as children.
	The rendering compoments (GL_Mesh, GL_Material, etc) are not copied.
	See the private member function, SoftCopyFrom(.). 
	*/
	void AddCloneOfNodeHierarchy(GL_SceneNode *n);

	/**
	Build a single Mesh model includes all the mesh in the hierarchy.
	It ignores line meshes (MT_LINES) and point meshes (MT_POINTS).
	*/
	void BuildSinglePolygonMesh(Mesh* out_mesh) const;

	/**
	Print 
	*/
	void PrintNodeHierarchy(std::ostream & out);


	/**
	Depth First Search
	*/
	void GetNodeHierarchyInDFS(std::vector<GL_SceneNode*> &out) ;
	void GetAllRenderableObjsInDFS(std::vector<GL_RenderableObj*> &out) ;

	const GL_SceneNode *FindChildByName(std::string node_name) const;
	bool IsAncestor(std::string node_name) const;
	bool IsAncestor(const GL_SceneNode * s) const;


	std::string name() const { return name_; }
	const GL_SceneNode * parent() const { return parent_; }

protected:
	/**
	It is used in AddCloneOfNodeHierarchy().
	*/
	void SoftCopyFrom(const GL_SceneNode *src);

protected:

	std::string name_;

	//////////////////////////////////////////////////
	// Transformation.
	TRS_Transf transf_;

	///////////////////////////////////////////////////
	// Renderable Objects.
	std::vector<mg::GL_RenderableObj*> renderable_objs_;

	///////////////////////////////////////////////////
	// Children.
	std::vector< std::unique_ptr<GL_SceneNode> > children_;

	GL_SceneNode* parent_;
};


class GL_SceneRoot : public GL_SceneNode
{
public:
	GL_SceneRoot()
	{
	}

};






};





