extern "C"{
//#include "libqhull_r.h"
//#include "qhull_ra.h"
}

#include "QHull.h"
#include "QhullFacetList.h"
#include "QhullVertexSet.h"
#include "QhullFacetSet.h"
#include "QhullRidge.h"
#include "BaseLib/CmlExt/CmlExt.h"
#include "BisectorSurface.h"

BisectorSurface::BisectorSurface()
{
	num_A_vertices_ = 0;
	num_B_vertices_ = 0;
	A_vertices_ = nullptr;
	B_vertices_ = nullptr;

	world_bounding_box_.push_back(cml::vector3d(-500, 0, -500));
	world_bounding_box_.push_back(cml::vector3d(-500, 0, +500));
	world_bounding_box_.push_back(cml::vector3d(+500, 0, +500));
	world_bounding_box_.push_back(cml::vector3d(+500, 0, -500));
	world_bounding_box_.push_back(cml::vector3d(-500, +500, -500));
	world_bounding_box_.push_back(cml::vector3d(-500, +500, +500));
	world_bounding_box_.push_back(cml::vector3d(+500, +500, +500));
	world_bounding_box_.push_back(cml::vector3d(+500, +500, -500));

	points_set_ = nullptr;
}

BisectorSurface::~BisectorSurface()
{
	if ( A_vertices_ != nullptr ) delete[] A_vertices_;
	if ( B_vertices_ != nullptr ) delete[] B_vertices_;
	A_vertices_ = nullptr;
	B_vertices_ = nullptr;
}

void 
BisectorSurface::SetVertices(const std::vector<cml::vector3d> &a, const std::vector<cml::vector3d> &b)
{

	if ( A_vertices_ != nullptr ) delete[] A_vertices_;
	if ( B_vertices_ != nullptr ) delete[] B_vertices_;
	A_vertices_ = nullptr;
	B_vertices_ = nullptr;

	num_A_vertices_ = (int)a.size();
	num_B_vertices_ = (int)b.size();

	A_vertices_ = new double[num_A_vertices_*3];
	B_vertices_ = new double[num_B_vertices_*3];

	for ( int i=0; i<num_A_vertices_; i++ )
	{
		A_vertices_[i*3+0] = a[i][0];
		A_vertices_[i*3+1] = a[i][1];
		A_vertices_[i*3+2] = a[i][2];
	}

	for ( int i=0; i<num_B_vertices_; i++ )
	{
		B_vertices_[i*3+0] = b[i][0];
		B_vertices_[i*3+1] = b[i][1];
		B_vertices_[i*3+2] = b[i][2];
	}
}

void 
BisectorSurface::SetVertices(int num_a, double *a_vertices, int num_b, double *b_vertices)
{
	if (num_A_vertices_ + num_B_vertices_ != num_a + num_b)
	{
		if (A_vertices_ != nullptr) delete[] A_vertices_;
		if (B_vertices_ != nullptr) delete[] B_vertices_;
		A_vertices_ = nullptr;
		B_vertices_ = nullptr;

		num_A_vertices_ = num_a;
		num_B_vertices_ = num_b;

		A_vertices_ = new double[num_A_vertices_ * 3];
		B_vertices_ = new double[num_B_vertices_ * 3];

	}

	memcpy((void*)A_vertices_, a_vertices, num_a*3*sizeof(double));
	memcpy((void*)B_vertices_, b_vertices, num_b*3*sizeof(double));
}



// refer the function, qh_printvdiagram in io.c in QHull libraray.
void
BisectorSurface::UpdateVoronoiDiagram(orgQhull::Qhull *qhull)
{
	bisector_faces_.clear();
	bisector_faces_sites_.clear();

	facetT *facetlist = qhull->firstFacet().getFacetT();
	setT *vertices;
	int totcount=0, numcenters;
	boolT isLower;
	qh_RIDGE innerouter = qh_RIDGEall;
	printvridgeT printvridge = qh_printvridge;
	qhT *qh = qhull->qh();

	vertices = qh_markvoronoi(qh, facetlist, 0, true, &isLower, &numcenters);

	vertexT *vertex;
	int vertex_i, vertex_n;

	FORALLvertices
	{
		vertex->seen= False;
		vertex->seen2 = True;
	}

	FOREACHvertex_i_(qh, vertices) 
	{
		if (vertex) 
		{

			if (qh->GOODvertex > 0 && qh_pointid(qh, vertex->point)+1 != qh->GOODvertex)
				continue;
			totcount += this->qh_eachvoronoi(qh, printvridge, vertex, !qh_ALL, innerouter, true);
		}
	}
	//totcount= qh_printvdiagram2 (0, printvridge, vertices, innerouter, True /* inorder*/);
}

int 
BisectorSurface::qh_eachvoronoi(qhT *qh, printvridgeT printvridge, vertexT *atvertex, boolT visitall, qh_RIDGE innerouter, boolT inorder) 
{
	boolT unbounded;
	int count;
	facetT *neighbor, **neighborp, *neighborA, **neighborAp;
	setT *centers;
	setT *tricenters= qh_settemp(qh, qh->TEMPsize);

	vertexT *vertex, **vertexp;
	boolT firstinf;
	unsigned int numfacets= (unsigned int)qh->num_facets;
	int totridges= 0;

	qh->vertex_visit++;
	atvertex->seen= True;
	if (visitall) {
		FORALLvertices
			vertex->seen= False;
	}
	FOREACHneighbor_(atvertex) {
		if (neighbor->visitid < numfacets)
			neighbor->seen= True;
	}
	FOREACHneighbor_(atvertex) {
		if (neighbor->seen) {
			FOREACHvertex_(neighbor->vertices) {
				if (vertex->visitid != qh->vertex_visit && !vertex->seen) {
					vertex->visitid= qh->vertex_visit;
					count= 0;
					firstinf= True;
					qh_settruncate(qh, tricenters, 0);
					FOREACHneighborA_(vertex) {
						if (neighborA->seen) {
							if (neighborA->visitid) {
								if (!neighborA->tricoplanar || qh_setunique(qh, &tricenters, neighborA->center))
									count++;
							}else if (firstinf) {
								count++;
								firstinf= False;
							}
						}
					}
					if (count >= qh->hull_dim - 1) {  /* e.g., 3 for 3-d Voronoi */
						if (firstinf) {
							if (innerouter == qh_RIDGEouter)
								continue;
							unbounded= False;
						}else {
							if (innerouter == qh_RIDGEinner)
								continue;
							unbounded= True;
						}
						totridges++;
						trace4((qh, qh->ferr, 4017, "qh_eachvoronoi: Voronoi ridge of %d vertices between sites %d and %d\n",
							count, qh_pointid(qh, atvertex->point), qh_pointid(qh, vertex->point)));
						
						if (printvridge) {
							if (inorder && qh->hull_dim == 3+1) /* 3-d Voronoi diagram */
								centers= qh_detvridge3 (qh, atvertex, vertex);
							else
								centers= qh_detvridge(qh, vertex);

							// (*printvridge) (fp, atvertex, vertex, centers, unbounded);
							{
								std::pair<int, int> sites;
								sites.first = qh_pointid(qh, atvertex->point);
								sites.second = qh_pointid(qh, vertex->point);

								// Collect olny the faces that are between two different objects.
								if ( ( isVertexOfA( sites.first ) && isVertexOfB( sites.second ) )
									||	( isVertexOfB( sites.first ) && isVertexOfA( sites.second ) ) 
									) 
								{
									bool upperdelaunay = false;
									facetT *facet, **facetp;
									std::vector<int> ridge;
									FOREACHfacet_(centers)
									{
										/* See io_r.c of qhull library.
										all facet->visitid == 0 if vertex_at_infinity
										                   == index of Voronoi vertex
										                   >= qh.num_facets if ignored
										*/
										
										if ( facet->upperdelaunay )
										{
											upperdelaunay = true;
											break;
										}
										if ( facet->visitid > 0 && facet->visitid <= (int)voronoi_vertices_.size() )
										{
											ridge.push_back(facet->visitid-1);
										}
									}

									if ( !upperdelaunay && ridge.size()>=3 )
									{
										bisector_faces_.push_back(ridge);
										bisector_faces_sites_.push_back(sites);
									}
								}
							}

							qh_settempfree(qh, &centers);
						}
					}
				}
			}
		}
	}
	FOREACHneighbor_(atvertex)
		neighbor->seen= False;
	qh_settempfree(qh, &tricenters);
	return totridges;
}

bool
BisectorSurface::isVertexOfA(int i) const
{
	if ( i < (int)num_A_vertices_ ) return true;
	return false;
}

bool
BisectorSurface::isVertexOfB(int i) const
{
	if ( (i - (int)num_A_vertices_) >= 0 && 
		(i - (int)num_A_vertices_) < (int)num_B_vertices_ ) return true;
	return false;
}

void
BisectorSurface::Update()
{
	
	{
		int num_point_set = (int)num_A_vertices_ + (int)num_B_vertices_ + (int)world_bounding_box_.size();	
		if ( points_set_ != nullptr ) delete[] points_set_;
		points_set_ = new double[num_point_set*3];

		memcpy((void*)points_set_, (void*)A_vertices_, num_A_vertices_*3*sizeof(double));
		
		memcpy((void*)(points_set_+(num_A_vertices_*3)), (void*)B_vertices_, num_B_vertices_*3*sizeof(double));

		// bouding-box corners
		for ( int i=0; i<(int)world_bounding_box_.size(); i++ )
		{
			points_set_[(int)(num_A_vertices_+num_B_vertices_)*3 + i*3+0] = world_bounding_box_[i][0];
			points_set_[(int)(num_A_vertices_+num_B_vertices_)*3 + i*3+1] = world_bounding_box_[i][1];
			points_set_[(int)(num_A_vertices_+num_B_vertices_)*3 + i*3+2] = world_bounding_box_[i][2];
		}


		voronoi_vertices_.clear();

		// voronoi
		orgQhull::Qhull qhull;
		qhull.runQhull("", 3, num_point_set, points_set_, "v");
		//qh_triangulate_facet(qhull.qh(), qhull.qh()->facet_list, &qhull.qh()->vertex_list);

		//qhull.runQhull("", 3, num_point_set, points_set_, "v QJ0.001");
		for ( orgQhull::QhullFacet facet = qhull.firstFacet();
			facet != qhull.endFacet();
			facet=facet.next() )
		{
			orgQhull::QhullPoint p = facet.voronoiVertex();
			voronoi_vertices_.push_back(cml::vector3d(p[0], p[1], p[2]));
		}





		UpdateVoronoiDiagram(&qhull);

		// normals
		bisector_faces_normals_.resize(bisector_faces_.size());
		for ( int i=0; i<(int)bisector_faces_.size(); i++ )
		{
			if ( (int)bisector_faces_[i].size() > 2 )
			{
				int v_i = bisector_faces_[i][0];//-1;
				int v_j = bisector_faces_[i][1];//-1;
				int v_k = bisector_faces_[i][2];//-1;
				if ( v_i < 0 || v_j < 0 || v_k < 0 ) continue;

				bisector_faces_normals_[i] = cml::normalize( 
						cml::cross((voronoi_vertices_[v_j]-voronoi_vertices_[v_i]),
									(voronoi_vertices_[v_k]-voronoi_vertices_[v_j]))
							);

			}

		}


	}


}