#include <math.h>
#include "BaseLib/CmlExt/CmlCamera.h"
#include "GL/glew.h"

#define MAX(x,y) ( ((x)>(y)) ? (x) : (y) )
#define MIN(x,y) ( ((x)<(y)) ? (x) : (y) )
#define SIGN(a,b) ((b) >= 0.0 ? fabs(a) : -fabs(a))


namespace cml
{
	bool GetPlaneCrossPoint(int inputX, int inputY, cml::vector3d const &planeP, cml::vector3d const &planeN, Camera const& camera, cml::vector3d &outP)
	{
		double *mv = camera.GetGLViewMatrix().data();
		double *p = camera.GetGLProjectionMatrix().data();

		cml::vector3d up0 = UnProjectWinP(inputX, inputY, 0, camera);
		cml::vector3d up1 = UnProjectWinP(inputX, inputY, 1, camera);


		// Ray
		cml::vector3d ray_dir = cml::normalize(up1 - up0);


		cml::vector3d planeP_s = planeP;// * camera.getScale();

										//if ( fabs(ray2[1]-ray1[1]) > 0.000001 )
		if ( fabs(cml::dot(planeN, ray_dir)) > 0.000001 )
		{
			outP = up0 + ( cml::dot(planeN, (planeP_s-up0)) / cml::dot(planeN, ray_dir) ) * ray_dir;
			return true;
		}
		return false;
	}



	double GetDistMouseAnd3dPoint(int win_x, int win_y, cml::vector3d const &p_3d, Camera const& camera)
	{
		double dist;

		// plane, parallal to caamera.
		cml::vector3d plane_n = cml::Rotate(camera.getRotation(), -1*cml::z_axis_3D()).normalize();
		cml::vector3d plane_p;

		GetPlaneCrossPoint(win_x, win_y, p_3d, plane_n, camera, plane_p);

		dist = cml::length(plane_p - p_3d);

		return dist;
	}








	bool GetPointOnRenderedObject(int win_mouse_x, int win_mouse_y, Camera const& camera, cml::vector3d &outP)
	{
		glFlush();
		GLint viewport[4];
		glGetIntegerv(GL_VIEWPORT,viewport);


		// Get the Z coordinate for the pixel location
		GLuint lDepth;
		glReadPixels( win_mouse_x, ( viewport[ 3 ] - win_mouse_y ), 
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

		double lDistance = lDepth * lDistTermInv;

		double *mv = camera.GetGLViewMatrix().data();
		double *p = camera.GetGLProjectionMatrix().data();

		outP = UnProjectWinP( win_mouse_x, win_mouse_y, lDistance, camera);

		/*gluUnProject( win_mouse_x, ( viewport[ 3 ] - win_mouse_y ), 
			lDistance, mv, p, viewport,
			&outP[ 0 ], &outP[ 1 ], &outP[ 2 ] );
*/
		return true;
	}




	cml::vector3d UnProjectWinP(int inputX, int inputY, double z, Camera const& camera)
	{
		double *mv = camera.GetGLViewMatrix().data();
		double *p = camera.GetGLProjectionMatrix().data();


		// gl viewport
		GLint gl_viewport[4];
		glGetIntegerv(GL_VIEWPORT, gl_viewport);

		// viewport matrix
		cml::matrix44d viewport_matrix;
		cml::matrix_viewport(
			viewport_matrix, 
			(double)gl_viewport[0], 
			(double)gl_viewport[0]+gl_viewport[2], 
			(double)gl_viewport[1], 
			(double)gl_viewport[1]+gl_viewport[3], z_clip_neg_one);


		// 3d points in Viewport Coordinates
		cml::vector3d p0(inputX, gl_viewport[3]-inputY, z);

		// Unprojection
		// 3d poiints in World Coordinates
		cml::vector3d out_p = cml::unproject_point(
			camera.GetGLViewMatrix(),
			camera.GetGLProjectionMatrix(),
			viewport_matrix,
			p0);

		return out_p;
	}

};