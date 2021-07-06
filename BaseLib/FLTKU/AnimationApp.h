
#pragma once

#include <FL/Fl.H>
#include <FL/Fl_Window.H>
#include <FL/Fl_Light_Button.H>

#include "BaseLib/FLTKU/AnimationViewer.h"
#include "BaseLib/FLTKU/AnimationTimeController.h"
#include "BaseLib/FLTKU/AnimationBrowser.h"
#include "BaseLib/FLTKU/AnimationControlBox.h"

namespace mg
{

class AnimationApp : public Fl_Window
{
public:
	AnimationApp(const char *s=0);

	virtual void Init();

	AnimationViewer *ani_viewer_;
	AnimationBrowser *ani_browser_;
	AnimationTimeController *ani_time_control_;
	AnimationControlBox *ani_control_box_;

	Fl_Light_Button *align_btn_;
};


};