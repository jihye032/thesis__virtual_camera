
#pragma once
#include "BaseLib/CmlExt/CmlExt.h"

namespace cml
{

class Camera
{
public:

	enum { IN_ROTATION, IN_ROTATION_Y_UP, IN_ZOOM, IN_TRANS, IN_TRANS_Z, IN_FOV, IN_NONUNIFORM_ZOOM };

	Camera();

	void setGLMatrix(double m[16]);
	void getGLMatrix(double m[16]);

	void setTranslation(const cml::vector3d &v);
	void setRotation(const cml::quaterniond &q);
	void setRotation(const cml::vector3d &euler_xyz);
	void setZoom(double z);
	void setZoom(cml::vector3d z);
	void setZoom(double sx, double sy, double sz);

	void lookAt(cml::vector3d eye, cml::vector3d centor, cml::vector3d up);
	cml::matrix44d GetGLViewMatrix() const;
	cml::matrix44d GetGLViewMatrixInverse() const;
	cml::matrix44d GetGLProjectionMatrix() const;
	cml::matrix44d GetGLProjectionViewMatrix() const;

	void setFov(double f);
	double getFov() const;

	void setAspectRatio(double r);
	double getAspectRatio() const;

	void setOrthVolume(cml::vector3d volume) { orth_viewport_volume_min=-0.5*volume; orth_viewport_volume_max=0.5*volume; }
	void setOrthVolume(cml::vector3d volume_min, cml::vector3d volume_max) { orth_viewport_volume_min=volume_min; orth_viewport_volume_max=volume_max; }
	cml::vector3d getOrthVolumeSize() const { return orth_viewport_volume_max-orth_viewport_volume_min; }
	cml::vector3d getOrthVolumeMin() const { return orth_viewport_volume_min; }
	cml::vector3d getOrthVolumeMax() const { return orth_viewport_volume_max; }
	void setNearFar(double n, double f);
	double getNear() const;
	double getFar() const;

	cml::vector3d getTranslation() const;
	cml::quaterniond getRotation() const;
	cml::vector3d    getViewDirection() const;
	cml::vector3d	getZoom() const;

	cml::vector3d getTranslationForGL() const;
	cml::quaterniond getRotationForGL() const;

	// x1, y1 => previouse normalized mouse point, (0~1, 0~1).
	// x2, y2 => current normalized mouse point, (0~1, 0~1).
	void inputMouse(int button, double x1, double y1, double x2, double y2, double speedScale = -1.0f);
	void inputMouse(int button, double x1, double y1, double x2, double y2, cml::vector3d center, double speedScale = -1.0f);
	void setPivot(cml::vector3d p) { pivot = p; }


	// dx와 dy는 moust point 의 변화량. (y는 화면에서 높으수록 증가)
	void inputMouse(int button, int dx, int dy, double speedScale = -1.0f);

	void inputMouse(int button, int dx, int dy, int dz, double speedScale = -1.0f);

	void enableOrtho(bool f) { flag_ortho = f; }
	bool isOrtho() const { return flag_ortho; }

	Camera& operator=(const Camera& a);

protected:
	cml::vector3d projectToTrackBall(double x, double y);

protected:
	cml::vector3d cameraP;

	// camera 의 실제 물리적 orientation
	cml::quaterniond cameraQ;			
	
	//double cameraZoom;
	cml::vector3d cameraZoom;
	
	double fov;
	double aspect;
	bool flag_ortho;

	double _near, _far;

	double trackBallR;
	cml::vector3d orth_viewport_volume_min; 
	cml::vector3d orth_viewport_volume_max; 

	cml::vector3d pivot;
};




// Pickings
/**
@pararm win_x, win_y are integers.
*/
bool GetPlaneCrossPoint(int win_x, int win_y, cml::vector3d const &planeP, cml::vector3d const &planeN, cml::Camera const& camera, cml::vector3d &outP);

/**
@pararm win_x, win_y are integers.
*/
double GetDistMouseAnd3dPoint(int win_x, int win_y, cml::vector3d const &p_3d, Camera const& camera);

/**
@pararm win_x, win_y are integers.
*/
bool GetPointOnRenderedObject(int win_mouse_x, int win_mouse_y, Camera const& camera, cml::vector3d &outP);

// (win_x, win_z) is a 2D point in the window coordinate.
// z is an imaginary value, which means a distance from the viewer.
cml::vector3d UnProjectWinP(int win_x, int win_y, double z, Camera const& camera);


};