#pragma once

#include "BaseLib/CmlExt/CmlCamera.h"
#include "BaseLib/FLTKU/Animation.h"
#include "BaseLib/GL4U/GL_FrameBuffer.h"
#include "BaseLib/GL4U/GL_SceneGraph.h"
#include "BaseLib/Geometry/Mesh.h"
#include <memory>
#include "BisectorSurface.h"
#include <Leap/Leap.h>


//using namespace Leap;


class BisectorSurfaceTestAni3 : public mg::Animation, public Leap::Listener
{
public:
	BisectorSurfaceTestAni3();

	virtual void SetCurrentTime(mg::AniTime_ms t) override;
	virtual void Draw(mg::AnimationViewer *w) override;
	void DrawBisector();
	void DrawInSubViewport(int vp_x, int vp_y, int vp_w, int vp_h, int border_width);
	void DrawInSubViewport2(int vp_x, int vp_y, int vp_w, int vp_h, int border_width);

	cml::vector3d FindSubCameraPosition();

	//// Leap
	//virtual void onInit(const Leap::Controller&);
	//virtual void onConnect(const Leap::Controller&);
	//virtual void onDisconnect(const Leap::Controller&);
	//virtual void onExit(const Leap::Controller&);
	//virtual void onFrame(const Leap::Controller&);
	//virtual void onFocusGained(const Leap::Controller&);
	//virtual void onFocusLost(const Leap::Controller&);
	//virtual void onDeviceChange(const Leap::Controller&);
	//virtual void onServiceConnect(const Leap::Controller&);
	//virtual void onServiceDisconnect(const Leap::Controller&);
	//virtual void onServiceChange(const Leap::Controller&);
	//virtual void onDeviceFailure(const Leap::Controller&);
	//virtual void onLogMessage(const Leap::Controller&, Leap::MessageSeverity severity, int64_t timestamp, const char* msg);

protected:

	mg::Mesh mesh_B_;
	mg::Mesh mesh_joint_, mesh_joint_obj_;
	mg::Mesh mesh_cmr_;

	BisectorSurface b_surface_;

	const std::vector<cml::vector3d>&      b_vertices = b_surface_.voronoi_vertices();
	const std::vector< std::vector<int> >& b_faces = b_surface_.bisector_faces();
	const std::vector<cml::vector3d>&      b_normals = b_surface_.bisector_faces_normals();
	const std::vector< std::pair<int, int> >&      b_sites = b_surface_.bisector_faces_sites();

	int num_leap_hands_;
	cml::vector3d finger_joint[3];
	cml::vector3d finger_joint_p[3];
	cml::vector3d car_nor_, dot_nor_;

	cml::vector3d public_cmr_pos;;

	bool find_camera_ = false;
	int cmr_pos_id_ = 0;


public:
	cml::Camera camera1_;
	std::unique_ptr<mg::GL_SceneRoot> scene_root_;


};



