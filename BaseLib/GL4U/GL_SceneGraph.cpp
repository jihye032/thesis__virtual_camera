
#pragma once

#include "GL/glew.h"
#include "BaseLib/GL4U/GL_SceneGraph.h"
#include "BaseLib/GL4U/GL_ResourceManager.h"
#include "BaseLib/GL4U/GL_Mesh.h"
#include <algorithm>
#include <stack>

namespace mg
{

GL_SceneNode::GL_SceneNode()
{
	transf_.SetIndentity();
	parent_ = nullptr;
}

GL_SceneNode* 
GL_SceneNode::CreateChild()
{
	std::unique_ptr<GL_SceneNode> n(new GL_SceneNode);
	children_.push_back(std::move(n));
	children_.back()->parent_ = this;

	return children_.back().get();
}


void 
GL_SceneNode::RemoveChild(GL_SceneNode *c)
{
	for ( unsigned int i=0; i<children_.size(); i++ )
	{
		if ( children_[i].get() == c )
		{
			children_.erase(children_.begin()+i);
			break;
		}
	}
}

void
GL_SceneNode::AddCloneOfNodeHierarchy(GL_SceneNode *n)
{
	if ( !n ) return;

	std::stack<GL_SceneNode*> src_dfs;
	src_dfs.push(n);

	std::stack<GL_SceneNode*> des_dfs;
	des_dfs.push(CreateChild());

	while ( !src_dfs.empty() )
	{
		GL_SceneNode *src_node = src_dfs.top();
		GL_SceneNode *des_node = des_dfs.top();
		src_dfs.pop();
		des_dfs.pop();

		des_node->SoftCopyFrom(src_node);


		for ( int i=0; i<src_node->num_children(); i++ )
		{
			src_dfs.push(src_node->child(i));
			des_dfs.push(des_node->CreateChild());
		}
	}
}

void 
GL_SceneNode::SoftCopyFrom(const GL_SceneNode *src)
{
	this->name_ = src->name_; //TODO ??? Should it be a unique name??
	this->transf_ = src->transf_; //TODO ??? Should it be a unique name??
	this->renderable_objs_.clear();
	this->renderable_objs_.assign(src->renderable_objs_.begin(), src->renderable_objs_.end());
}



void 
GL_SceneNode::AddRenderableObj(mg::GL_RenderableObj* obj)
{
	renderable_objs_.push_back(obj);
}

void 
GL_SceneNode::AddRenderableObj(std::string name)
{
	GL_RenderableObj *o = GL_ResourceManager::singleton()->GetRenderableObj(name);
	if ( o == nullptr ) return;
	AddRenderableObj(o);
}



void
GL_SceneNode::BuildSinglePolygonMesh(Mesh* out_mesh) const
{
	std::stack<cml::matrix44d> transf_stack;
	transf_stack.push(this->transf_.GetMat44());

	std::stack<const GL_SceneNode*> node_dfs;
	node_dfs.push(this);

	while ( !node_dfs.empty() )
	{
		const GL_SceneNode *cur_node = node_dfs.top();
		node_dfs.pop();

		cml::matrix44d cur_transf = transf_stack.top();
		transf_stack.pop();

		for ( int i=0; i<cur_node->num_children(); i++ )
		{
			node_dfs.push(cur_node->child(i));
			transf_stack.push(cur_transf*cur_node->child(i)->transf().GetMat44());
		}

		Mesh tmp_mesh;
		for ( auto r_obj : cur_node->renderable_objs_ )
		{
			if (r_obj->mesh() != nullptr && 
				(r_obj->mesh()->mesh_type() == Mesh::MT_TRIANGLES ||
				 r_obj->mesh()->mesh_type() == Mesh::MT_POLYGONS) )
			{
				tmp_mesh.Merge(*r_obj->mesh());
			}
		}

		if ( tmp_mesh.num_vertices() > 0 )
		{
			tmp_mesh.TransformVertices(cur_transf);
			out_mesh->Merge(tmp_mesh);
		}
	}

	
}


void
GL_SceneNode::PrintNodeHierarchy(std::ostream &out)
{
	out << "Node name: " << this->name_ << std::endl;
	out << "Transf: " << this->transf_.ToString() << std::endl;

	for ( auto &o : this->renderable_objs_ )
	{
		out << "  Renderable Obj: " << std::endl;
		if ( o->mesh() )
		{
			out << "    Mesh Vertex Size: " << o->mesh()->num_vertices() << std::endl;
			out << "    Mesh Face Size: " << o->mesh()->num_faces() << std::endl;
		}
	}

	for ( auto &n : this->children_ )
	{
		n->PrintNodeHierarchy(out);
	}
}


void
GL_SceneNode::GetNodeHierarchyInDFS(std::vector<GL_SceneNode*> &out) 
{
	std::stack<GL_SceneNode*> dfs_stack;
	dfs_stack.push(this);

	while ( !dfs_stack.empty() )
	{
		GL_SceneNode* node = dfs_stack.top();
		dfs_stack.pop();

		out.push_back(node);
		
		for ( const auto &d : node->children_ )
		{
			dfs_stack.push(d.get());
		}
	}
}


void
GL_SceneNode::GetAllRenderableObjsInDFS(std::vector<GL_RenderableObj*> &out) 
{
	std::vector<GL_SceneNode*> node_list;
	
	GetNodeHierarchyInDFS(node_list);

	for ( auto &n : node_list )
	{
		for ( auto &r : n->renderable_objs_ )
		{
			out.push_back(r);
		}
	}
}

const GL_SceneNode *
GL_SceneNode::FindChildByName(std::string node_name) const
{
	std::stack<const GL_SceneNode*> dfs_stack;
	dfs_stack.push(this);

	while ( !dfs_stack.empty() )
	{
		const GL_SceneNode* node = dfs_stack.top();
		dfs_stack.pop();

		if ( node_name.compare(node->name()) == 0 ) return node;

		for ( const auto &d : node->children_ )
		{
			dfs_stack.push(d.get());
		}
	}

	return nullptr;
}

bool 
GL_SceneNode::IsAncestor(std::string node_name) const
{
	const GL_SceneNode *node = this;

	while ( node )
	{
		if ( node->name_.compare(node_name) == 0 ) return true;
		node = node->parent_;
	}
	return false;
}

bool 
GL_SceneNode::IsAncestor(const GL_SceneNode * s) const
{
	const GL_SceneNode *node = this;

	while ( node )
	{
		if ( node == s ) return true;
		node = node->parent_;
	}

	return false;
}

};





