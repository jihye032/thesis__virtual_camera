
#include"BaseLib/FLTKU/AnimationApp.h"

namespace mg
{

#define ANI_WIN_W 1000
#define ANI_WIN_H 600
#define RIGHT_SIDE_W 300
#define MENU_BUTTON_H 30
#define MENU_BUTTON_W 100
#define BOTTOM_ROWS 2
#define APP_W ANI_WIN_W+RIGHT_SIDE_W
#define APP_H ANI_WIN_H+BOTTOM_ROWS*MENU_BUTTON_H


void alignButtonCB(Fl_Widget *w, void *d);

AnimationApp::AnimationApp(const char *s) : Fl_Window(APP_W, APP_H, s)
{
	Init();
}

void
AnimationApp::Init()
{
	ani_viewer_ = new AnimationViewer(0, 0, ANI_WIN_W, ANI_WIN_H);
	ani_viewer_->end();

	ani_time_control_ = new AnimationTimeController(0, ANI_WIN_H, ANI_WIN_W, MENU_BUTTON_H);
	ani_time_control_->end();

	ani_control_box_ = new AnimationControlBox(ANI_WIN_W, ANI_WIN_H/2, RIGHT_SIDE_W, ANI_WIN_H/2);
	ani_control_box_->end();

	ani_browser_ = new AnimationBrowser(ANI_WIN_W, 0, RIGHT_SIDE_W, ANI_WIN_H/2);
	ani_browser_->end();
	ani_browser_->AddAniBrowserListener(ani_viewer_);
	ani_browser_->AddAniBrowserListener(ani_time_control_);
	ani_browser_->AddAniBrowserListener(ani_control_box_);

	align_btn_ = new Fl_Light_Button(0, ANI_WIN_H+MENU_BUTTON_H, MENU_BUTTON_W, MENU_BUTTON_H, "Align");
	align_btn_->box(FL_SHADOW_BOX);
	align_btn_->down_box(FL_FLAT_BOX);
	align_btn_->callback(alignButtonCB, this);

	resizable(ani_viewer_);
	end();
}


void alignButtonCB(Fl_Widget *w, void *d)
{
	if ( ((AnimationApp*)d)->ani_viewer_->flag_align_in_grid() )
	{
		((AnimationApp*)d)->ani_viewer_->flag_align_in_grid(false);
		((Fl_Light_Button*)w)->value(false);
		((AnimationApp*)d)->ani_viewer_->redraw();
	}
	else
	{
		((AnimationApp*)d)->ani_viewer_->flag_align_in_grid(true);
		((Fl_Light_Button*)w)->value();
		((AnimationApp*)d)->ani_viewer_->redraw();
	}
}



};







