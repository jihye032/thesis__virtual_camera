
#pragma once

#include "BaseLib/CmlExt/CmlExt.h"
#include "BaseLib/Geometry/AABox.h"
#include <stdarg.h>
#include <string>

///////////////////////////////////////////
// Class: Mesh
// This class represents general geometry object as a mesh model.

namespace mg
{

class Mesh
{
public:
	enum Mesh_Type {MT_UNKNOWN, MT_POINTS, MT_LINES, MT_TRIANGLES, MT_POLYGONS};

	Mesh();
	Mesh(const Mesh &in_mesh);
	virtual ~Mesh();
	void Clear();

	// Create Premitives.
	void CreateQuad(double width=1, double depth=1);
	void CreateBox(double width=1, double height=1, double depth=1);
	void CreateCylinder(cml::vector3d bottom_center, cml::vector3d top_center, double radius=1, int slices=32, int stacks=1);
	void CreateCylinder(double height=1, double radius=1, int slices=32, int stacks=1);
	void CreateOpenedCylinder(double height=1, double radius=1, int slices=32, int stacks=1);
	void CreateSphere(double radius=1);
	void CreateHemisphere(double radius=1);
	void CreateCapsule(cml::vector3d bottom_center, cml::vector3d top_center, double radius=1, int slices=32, int stacks=1);
	void CreateCapsule(double cylinder_height=1, double radius=1);
	void CreateHeadModel(double radius=1);

	// Transform.
	void TranslateVertices(cml::vector3d t);
	void RotateVertices(cml::quaterniond q, cml::vector3d center=cml::vector3d(0, 0, 0));
	void ScaleUniformlyVertices(double s, cml::vector3d center=cml::vector3d(0, 0, 0));
	void ScaleVertices(cml::vector3d s, cml::vector3d center=cml::vector3d(0, 0, 0));
	void TransformVertices(const cml::matrix44d &t);
	void TransformVertices(const cml::matrix33d &t);

	// Assign, Merge
	virtual void Assign(const Mesh& mesh);
	virtual void Merge(const Mesh& mesh);

	// Calculating Normal Vector.
	cml::vector3d CalculFaceNormal(int face_id) const;
	cml::vector3d CalculVertexNormal(int vertex_id) const;
	void UpdateNormalVectorsBasedOnFace();
	void UpdateNormalVectorsBasedOnVertex();

	//
	void Triangulate();

	// Axis Aligned Bounding Box
	AABox CalculAABB() const;



	///////////////////////////////////////////////////////////////////////
	//// Set Properties. 
	void mesh_type(Mesh_Type t) { mesh_type_=t; }

	/**
	@param num # of 3D vertices.
	@param data The size of data must be 3*num
	*/
	void SetVertices(int num, double *data=0);

	/**
	@param num # of 3D normal vectors.
	@param data The size of data must be 3*num
	*/
	void SetNormals(int num, double *data=0);

	/**
	@param num # of 2D uv vectors.
	@param data The size of data must be 2*num
	*/
	void SetUVs(int num, double *data=0);

	/**
	@param num # of 4D color vectors.
	@param data The size of data must be 4*num
	*/
	void SetColors(int num, double *data=0);

	// Set Sizes and Allocate
	void SetVertexUvNormalColorFaceSizes(int num_vertices, int num_uvs, int num_normals, int num_colors, int num_faces);

	// Set indivisually
	inline void vertex(int id, const cml::vector3d &v) { vertices_[id * 3 + 0] = v[0]; vertices_[id * 3 + 1] = v[1]; vertices_[id * 3 + 2] = v[2]; }
	inline void normal(int id, const cml::vector3d &n) { normals_[id * 3 + 0] = n[0]; normals_[id * 3 + 1] = n[1]; normals_[id * 3 + 2] = n[2]; }
	inline void uv(int id, double u, double v) { uvs_[id * 2 + 0] = u; uvs_[id * 2 + 1] = v; }
	inline void color(int id, const cml::vector4d &c) { colors_[id * 4 + 0] = c[0]; colors_[id * 4 + 1] = c[1]; colors_[id * 4 + 2] = c[2]; colors_[id * 4 + 3] = c[3]; }

	// Set uniformly
	void SetAllNormals(cml::vector3d unified_normal);
	void SetAllUVs(double u, double v);
	void SetAllColors(cml::vector4d color);



	///////////////////////////////////////////////////////////////////////
	//// Set Face
	//// A face is a polygon.

	/**
	Set total face numbers, and allocate necessray memories.
	*/
	void SetNumFaces(int num);

	/**
	Reset the number of vertices of the face of face_id.
	And all the face vertex info are reset.
	After calling this function, vertex info values must be defined
	by using some of following functions.
	SetFaceVertexIds, SetFaceNormalIds, SetFaceUvIds, SetFaceColorIds.
	*/
	void SetFaceSize(int face_id, int face_size);

	void SetFaceVertexIds(int face_id, ...);
	void SetFaceNormalIds(int face_id, ...);
	void SetFaceUvIds(int face_id, ...);
	void SetFaceColorIds(int face_id, ...);


	/**
	Set the face of face_id with vertex and normal indices.
	@param face_id The index of the face
	@param face_size The number of vertices of the face
	@param ... 1'st vertex id, 1'st normal id, ..., n'th vertex id, n'th normal id, 
	           n is the number of points in the face (n==face_size).
	*/
	void SetFaceSizeVertexAndNormalIds(int face_id, int face_size, ...);

	
	/**
	Set the face of face_id with vertex, normal and uv indices.
	@param face_id The index of the face
	@param face_size The number of vertices of the face
	@param ... 1'st vertex id, 1'st normal id, 1'st uv id, ..., n'th vertex id, n'th normal id, n'th uv id
	           n is the number of points in the face (n==face_size).
	*/
	void SetFaceSizeVertexNormalUvIds(int face_id, int face_size, ...);




	///////////////////////////////////////////////////////////////////////
	//// Get Properties. 
	inline Mesh_Type mesh_type() const { return mesh_type_; }
	inline int num_vertices() const { return num_vertices_; }
	inline int num_uvs() const { return num_uvs_; }
	inline int num_normals() const { return num_normals_; }
	inline int num_faces() const { return num_faces_; }
	inline int num_colors() const { return num_colors_; }
	
	inline double*  vertices() const { return vertices_; }
	inline double*  uvs() const { return uvs_; }
	inline double*  normals() const { return normals_; }
	inline double*  colors() const { return colors_; }
	inline int*     face_sizes() const { return face_sizes_; }
	inline int**    vertices_of_faces() const { return face_vertex_ids_; }
	inline int**    uvs_of_faces() const { return face_uv_ids_; }
	inline int**    normals_of_faces() const { return face_normal_ids_; }

	inline std::pair<double, double> uv(int id) const { return std::make_pair(uvs_[id*2+0], uvs_[id*2+1]); }
	inline cml::vector3d vertex(int id)              const { return cml::vector3d(vertices_[id*3+0], vertices_[id*3+1], vertices_[id*3+2]); }
	inline cml::vector3d normal(int id)              const { return cml::vector3d(normals_[id*3+0], normals_[id*3+1], normals_[id*3+2]); }
	inline cml::vector4d color(int id)              const { return {colors_[id*4+0], colors_[id*4+1], colors_[id*4+2], colors_[id*4+3]}; }

	inline int        face_size(int face_id)          const { return face_sizes_[face_id]; }
	inline const int* face_vertex_ids(int face_id) const { return face_vertex_ids_[face_id]; }
	inline const int* face_uv_ids(int face_id)     const { return face_uv_ids_[face_id]; }
	inline const int* face_normal_ids(int face_id) const { return face_normal_ids_[face_id]; }
	inline const int* face_color_ids(int face_id)  const { return face_color_ids_[face_id]; }

	inline bool          has_bone() const { return flag_has_bone_; }
	inline int           max_bone_num_per_vertex() const { return max_bone_num_per_vertex_; }
	inline const int*    bone_ids(int vertex_id) const { return bone_ids_[vertex_id]; }
	inline       int*    bone_ids(int vertex_id)       { return bone_ids_[vertex_id]; }
	inline const double* bone_weights(int vertex_id) const { return bone_weights_[vertex_id]; }

	
	///////////////////////////////////////////////////////////////////////
	//// Bone 
	void UseBoneWeight(bool f=true);
	void UnuseBoneWeight();
	
	// Set 'i'th bone_id and weight for 'vertex_id'th vertex.
	inline void SetBoneIdAndWeight(int vertex_id, int i, int bone_id, double w) { bone_ids_[vertex_id][i] = bone_id; bone_weights_[vertex_id][i] = w; }
	
	// Set 'i'th bone_id for all vertices
	void SetBoneIdAndWeightForAllVertices(int i, int bone_id, double w);


	///////////////////////////////////////////////////////////////////////
	//// File IO
	void WriteObjFormatStream(std::ostream &out, std::string group_name="", int v_offset=0, int vt_offset=0, int vn_offset=0) const;
	void WriteObjFormatStreamV(std::ostream &out) const;	// Vertices.
	void WriteObjFormatStreamVT(std::ostream &out) const;	// UVs.
	void WriteObjFormatStreamVN(std::ostream &out) const;	// Normals.
	void WriteObjFormatStreamG(std::ostream &out, std::string group_name) const;	// Group name.
	void WriteObjFormatStreamF(std::ostream &out, int v_offset=0, int vt_offset=0, int vn_offset=0) const;		// Faces.
	void ReadObjFormatStream(std::istream &in);
	void ReadObjFormatFile(std::string file);

private:
	void SetZero();

	// Parameter: face_id, 1'st vertex id, 1'st normal id, ..., n'th vertex id, n'th normal id
	//            n is the number of points in the face (face_sizes_[face_id]).
	void SetFaceVertexAndNormalIds(int face_id, ...);

	// Parameter: face_id, 1'st vertex id, 1'st normal id, 1'st uv id, ..., n'th vertex id, n'th normal id, n'th uv id
	//            n is the number of points in the face (face_sizes_[face_id]).
	void SetFaceVertexNormalUvIds(int face_id, ...);


	

	

protected:
	Mesh_Type mesh_type_;

	int num_vertices_;
	int num_uvs_;
	int num_normals_;
	int num_faces_;
	int num_colors_;

	double *vertices_;
	double *uvs_;
	double *normals_;
	double *colors_;
	int *face_sizes_;
	int **face_vertex_ids_;
	int **face_uv_ids_;
	int **face_normal_ids_;
	int **face_color_ids_;

	// For skinning. 
	// A vertex can be connected up to 'max_bone_num_per_vertex_' boneds.
	// If flag_has_bone_ is true, bone_ids_'s size (and bone_weight's size) must be num_vertices_*max_bone_num_per_vertex_
	bool flag_has_bone_;
	static const int max_bone_num_per_vertex_ = 4;
	int** bone_ids_;
	double** bone_weights_;
};


};