#pragma once

#include "BaseLib/CmlExt/CmlCamera.h"
#include "BaseLib/FLTKU/Animation.h"
#include "BaseLib/GL4U/GL_FrameBuffer.h"
#include "BaseLib/GL4U/GL_SceneGraph.h"
#include "BaseLib/GL4U/GL_Mesh.h"
#include "BaseLib/Geometry/Line.h"
#include "BaseLib/Geometry/Mesh.h"
#include <memory>
#include "BisectorSurface.h"
#include <Leap/Leap.h>
#include "FL/fl.h"


//using namespace Leap;


class BisectorSurfaceTestAni5 : public mg::Animation, public Leap::Listener
{
public:
	BisectorSurfaceTestAni5();

	

	
	virtual void SetCurrentTime(mg::AniTime_ms t) override;
	virtual void Draw(mg::AnimationViewer *w) override;
	void DrawBisector();
	void DrawInSubViewport(int vp_x, int vp_y, int vp_w, int vp_h, int border_width);
	void DrawInSubViewport2(int vp_x, int vp_y, int vp_w, int vp_h, int border_width);
	void DrawInSubViewport3(int vp_x, int vp_y, int vp_w, int vp_h, int border_width);

	cml::vector3d FindSubCameraPosition();
	cml::vector3d FindFignerSubCameraPosition();

	//// Leap
	virtual void onInit(const Leap::Controller&);
	virtual void onConnect(const Leap::Controller&);
	virtual void onDisconnect(const Leap::Controller&);
	virtual void onExit(const Leap::Controller&);
	virtual void onFrame(const Leap::Controller&);
	virtual void onFocusGained(const Leap::Controller&);
	virtual void onFocusLost(const Leap::Controller&);
	virtual void onDeviceChange(const Leap::Controller&);
	virtual void onServiceConnect(const Leap::Controller&);
	virtual void onServiceDisconnect(const Leap::Controller&);
	virtual void onServiceChange(const Leap::Controller&);
	virtual void onDeviceFailure(const Leap::Controller&);
	virtual void onLogMessage(const Leap::Controller&, Leap::MessageSeverity severity, int64_t timestamp, const char* msg);

	virtual int HandleFLTK(mg::AnimationViewer *w, int event) override 
	{ 
		if (event == FL_SHORTCUT && Fl::event_key() == '1')
		{
			flag_subview_ = !flag_subview_;
			NotifyAnimationUpdated();
			return 1;
		}
		else if (event == FL_SHORTCUT && Fl::event_key() == '2')
		{
			flag_draw_bisector_ = !flag_draw_bisector_;
			NotifyAnimationUpdated();
			return 1;
		}
		return mg::Animation::HandleFLTK(w, event);
	}
protected:

	bool flag_subview_;
	bool flag_draw_bisector_;
	mg::Mesh mesh_B_;
	mg::Mesh mesh_joint_obj_;
	mg::Mesh mesh_joint_obj_T_;


	BisectorSurface b_surface_;

	int num_leap_hands_;
	bool contact_obj_;


	cml::vector3d finger_joint[4];
	cml::vector3d finger_last_;
	cml::vector3d finger_pos_;

	cml::vector3d pos_;

	cml::Camera *cmr_;					// main Camera

	cml::vector3d cmr_pos2_;
	cml::vector3d cmr_pos2_last_;

	cml::vector3d cmr_pos_;				// now camera position
	cml::vector3d cmr_pos_last_;		// last camera's position 

	cml::vector3d cnt_pos_;		// 카메라가 봐야할 위치(두 물체 사이의 중심)

	cml::vector3d mesh_pos_;
	bool turn_num;
	 

	
	

public:
	cml::Camera camera1_;
	cml::Camera camera2_;
	std::unique_ptr<mg::GL_SceneRoot> scene_root_;
	std::unique_ptr<mg::GL_SceneRoot> scene_skydome_;
	std::unique_ptr<mg::GL_SceneRoot> scene_playground_;
	std::unique_ptr<mg::GL_SceneRoot> scene_finger_;
	std::unique_ptr<mg::GL_SceneRoot> scene_object_;

	mg::GL_Mesh *mesh_;

};



