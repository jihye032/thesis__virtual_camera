
#pragma once

#include <FL/Fl.H>
#include <FL/Fl_Window.H>
#include <FL/Fl_Button.H>
#include <FL/Fl_Value_Slider.H>

#ifdef WIN32
#include <windows.h>
#endif
#include <vector>

#include "BaseLib/FLTKU/AnimationTimeListener.h"
#include "BaseLib/FLTKU/AnimationBrowserListener.h"
#include "BaseLib/FLTKU/AnimationListener.h"

namespace mg
{

class AnimationTimeController : public Fl_Window, public AnimationBrowserListener, public AnimationListener
{
public:
	enum PlayMode {ONE_TIME, ONE_TIME_AND_REWIND, REPEAT, TO_INFINITE};

	AnimationTimeController(int x, int y, int w, int h, const char *s=0);

	// play
	virtual void PlayToggle();
	virtual bool flag_play() const { return flag_play_; }
	virtual void SetPlayFPS_ms(int fps);
	virtual void SetCurrentTime_ms(AniTime_ms t, bool redrawing=true);
	virtual void SetFinalAniTime_ms(AniTime_ms t);
	virtual AniTime_ms GetCurrentTime_ms() const { return current_time_ms_; }
	void play_mode(PlayMode pm) { play_mode_ = pm; }
	PlayMode play_mode() const { return play_mode_; }

	virtual void AddAniFrameListener(AnimationTimeListener* a) { time_listeners_.push_back(a); }

	// from AnimationBrowserListener
	virtual void BrowserSelectionsChanged(const AnimationList& ani_list);
	virtual void BrowserItemsChanged(const AnimationList& ani_list);

	// from AnimationListener
	virtual void AnimationChanged(const Animation *ani);

	
protected:
	void UpdateSliderBarRange();
	void UpdateSliderBarCursor();
	static void StaticResetBtnCB(Fl_Widget *w, void *d);
	static void StaticPlayBtnCB(Fl_Widget *w, void *d);
	static void StaticTimeSliderCB(Fl_Widget *w, void *d);
	void ResetBtnCB();
	void PlayBtnCB();
	void TimeSliderCB();

	void NotifyTimeReset();
	void NotifyTimeChanged();
	void NotifyPlayStarted();
	void NotifyPlayStopped();

	static void StaticIdleCB(void *d);
	void IdleCB();


protected:
	int play_fps_;
	bool flag_play_;
	PlayMode play_mode_;
	AniTime_ms current_time_ms_;
	AniTime_ms final_ani_time_ms_;

	//double time_delay_;

	class Time_Slider : public Fl_Value_Slider
	{
	public:
		Time_Slider(int x, int y, int w, int h, const char *L=0) : Fl_Value_Slider(x, y, w, h, L) 
		{
		};
		virtual int format(char* buffer) override
		{
			return sprintf_s(buffer, 128, "%.2f", value()/1000);
		}
	};

	Fl_Button *play_btn_;
	Fl_Button *reset_btn_;
	Time_Slider *time_slider_ms_;

	std::vector<AnimationTimeListener*> time_listeners_;
};

};


