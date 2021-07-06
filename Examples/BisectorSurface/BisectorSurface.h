#pragma once
#include "Qhull.h"
#include "BaseLib/CmlExt/CmlExt.h"

class BisectorSurface
{
public:
	BisectorSurface();
	~BisectorSurface();
	
	void SetVertices(const std::vector<cml::vector3d> &a, const std::vector<cml::vector3d> &b);
	void SetVertices(int num_a, double *a_vertices, int num_b, double *b_vertices);

	cml::vector3d GetVertexOfA(int i) { return cml::vector3d(A_vertices_[i*3+0], A_vertices_[i * 3 + 1], A_vertices_[i * 3 + 2]);  }
	cml::vector3d GetVertexOfB(int i) { return cml::vector3d(B_vertices_[i * 3 + 0], B_vertices_[i * 3 + 1], B_vertices_[i * 3 + 2]); }
	
	void Update();

	const std::vector<cml::vector3d>& voronoi_vertices() const { return voronoi_vertices_; }
	const std::vector< std::vector<int> >& bisector_faces() const { return bisector_faces_; }
	const std::vector<cml::vector3d>& bisector_faces_normals() const { return bisector_faces_normals_; }
	const std::vector<std::pair<int, int> >& bisector_faces_sites() const { return bisector_faces_sites_; }


protected:
	void UpdateVoronoiDiagram(orgQhull::Qhull *qhull);
	int  qh_eachvoronoi(qhT *qh, printvridgeT printvridge, vertexT *atvertex, boolT visitall, qh_RIDGE innerouter, boolT inorder);
	bool isVertexOfA(int i) const;
	bool isVertexOfB(int i) const;

protected:
	std::vector<cml::vector3d> voronoi_vertices_;
	std::vector< std::pair<int, int> > bisector_faces_sites_;	// two vertices on the delaunay mesh corresponding the voronoi cell.
	std::vector< std::vector<int> > bisector_faces_;
	std::vector<cml::vector3d> bisector_faces_normals_;

	int num_A_vertices_, num_B_vertices_;
	double *A_vertices_;
	double *B_vertices_;
	std::vector<cml::vector3d> world_bounding_box_;	// 8 vertices for each corner.

	double *points_set_;
};
