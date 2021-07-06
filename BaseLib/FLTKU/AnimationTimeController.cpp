

#include "BaseLib/FLTKU/AnimationTimeController.h"

#include <algorithm>
#include <chrono>

namespace mg
{
	// Internally, we use std::chrono times for better accuracy 
	// Followings are the global variables. They are used in SetPlayFPS_ms, SetCurrentTime_ms, SetFinalAniTime_ms, PlayToggle and IdleCB.
	static std::chrono::milliseconds g_acc_cur_time;
	static std::chrono::milliseconds g_acc_final_time;
	static std::chrono::time_point<std::chrono::steady_clock> g_acc_idle_pre_time;
	static std::chrono::duration<double, std::milli> g_acc_time_interval;

AnimationTimeController::AnimationTimeController(int x, int y, int w, int h, const char *s) 
	: Fl_Window(x, y, w, h, s)
{
	play_mode_ = REPEAT;
	flag_play_ = false;

	reset_btn_ = new Fl_Button(0, 0, 30, h, "@<|");
	reset_btn_->box(FL_SHADOW_BOX);
	reset_btn_->down_box(FL_FLAT_BOX);
	reset_btn_->callback((Fl_Callback*)StaticResetBtnCB, (void*)(this));

	play_btn_ = new Fl_Button(30, 0, 30, h, "@>");
	play_btn_->box(FL_SHADOW_BOX);
	play_btn_->down_box(FL_FLAT_BOX);
	play_btn_->callback((Fl_Callback*)StaticPlayBtnCB, (void*)(this));
	


	time_slider_ms_ = new Time_Slider(60, 0, w-60, h);
	time_slider_ms_->type(5);
	time_slider_ms_->box(FL_SHADOW_BOX);
	time_slider_ms_->callback((Fl_Callback*)StaticTimeSliderCB, (void*)(this));

	SetPlayFPS_ms(60);
	SetFinalAniTime_ms(1000);
	SetCurrentTime_ms(0);
	

	UpdateSliderBarRange();
	UpdateSliderBarCursor();

	resizable(time_slider_ms_);

	end();
}

void
AnimationTimeController::UpdateSliderBarRange()
{
	time_slider_ms_->minimum(0);
	time_slider_ms_->maximum(final_ani_time_ms_);
}

void
AnimationTimeController::UpdateSliderBarCursor()
{
	time_slider_ms_->value(current_time_ms_);
}

void
AnimationTimeController::SetPlayFPS_ms(int fps)
{
	play_fps_ = fps;
	time_slider_ms_->step((int)(1000 / fps));
	g_acc_time_interval = std::chrono::duration<double, std::milli>(1000./fps);

}

void
AnimationTimeController::SetCurrentTime_ms(AniTime_ms t_ms, bool redrawing)
{
	if (play_mode_ == TO_INFINITE && current_time_ms_ < t_ms)
	{
		SetFinalAniTime_ms(t_ms);
	}

	g_acc_cur_time = std::chrono::milliseconds(t_ms);

	current_time_ms_ = t_ms;

	if (redrawing)
	{
		UpdateSliderBarCursor();
	}
	NotifyTimeChanged();
}


void
AnimationTimeController::SetFinalAniTime_ms(AniTime_ms t_ms)
{
	g_acc_final_time = std::chrono::milliseconds(t_ms);

	final_ani_time_ms_ = t_ms;
	UpdateSliderBarRange();
}


void 
AnimationTimeController::BrowserSelectionsChanged(const AnimationList& ani_list)
{
	static AnimationList cur_ani_list;
	
	// remove priviouse animations in the time_listeners list. 
	if ( cur_ani_list.size() > 0 )
	{
		for ( int i=0; i<(int)cur_ani_list.size(); i++ )
		{
			std::vector<AnimationTimeListener*>::iterator iter 
				= std::find(time_listeners_.begin(), time_listeners_.end(), cur_ani_list[i]);

			if ( iter != time_listeners_.end() )
				time_listeners_.erase(iter);
		}
	}

	// set new cur_ani_list.
	cur_ani_list.assign(ani_list.cbegin(), ani_list.cend());


	// add new animations to the time_listeners_ list
	for ( int i=0; i<(int)cur_ani_list.size(); i++ )
	{
		time_listeners_.push_back(cur_ani_list[i]);
	}

	
	// update slider size and time range.
	SetFinalAniTime_ms(ani_list.GetMaxFinalAniTime_ms());
	

	if ( !ani_list.empty() )
	{
		SetCurrentTime_ms(ani_list.front()->cur_time_ms());
	}

}

void 
AnimationTimeController::BrowserItemsChanged(const AnimationList& ani_list)
{
}


void 
AnimationTimeController::AnimationChanged(const Animation *ani)
{
	if ( ani==0 ) return;

	if (final_ani_time_ms_ < ani->final_ani_time_ms() )
	{
		SetFinalAniTime_ms(ani->final_ani_time_ms());
	}
}


void
AnimationTimeController::NotifyTimeReset()
{
	for ( int i=0; i<(int)time_listeners_.size(); i++ )
	{
		time_listeners_[i]->TimeReset();
	}
}


void
AnimationTimeController::NotifyTimeChanged()
{
	for ( int i=0; i<(int)time_listeners_.size(); i++ )
	{
		time_listeners_[i]->TimeChanged(current_time_ms_);
	}
}

void
AnimationTimeController::NotifyPlayStarted()
{

	for (int i = 0; i < (int)time_listeners_.size(); i++)
	{
		time_listeners_[i]->PlayStarted(current_time_ms_);
	}
}


void
AnimationTimeController::NotifyPlayStopped()
{

	for (int i = 0; i < (int)time_listeners_.size(); i++)
	{
		time_listeners_[i]->PlayStopped(current_time_ms_);
	}
}








//////////////////////////////////////////////////////////////
// events

void
AnimationTimeController::StaticResetBtnCB(Fl_Widget *w, void *d)
{
	((AnimationTimeController*)d)->ResetBtnCB();

}
void
AnimationTimeController::StaticPlayBtnCB(Fl_Widget *w, void *d)
{
	((AnimationTimeController*)d)->PlayBtnCB();
}

void
AnimationTimeController::StaticTimeSliderCB(Fl_Widget *w, void *d)
{
	((AnimationTimeController*)d)->TimeSliderCB();
}

void
AnimationTimeController::ResetBtnCB()
{
	flag_play_ = false;
	play_btn_->label("@>");
	SetCurrentTime_ms(0);
	NotifyTimeReset();
}

void
AnimationTimeController::PlayBtnCB()
{
	PlayToggle();
}

void
AnimationTimeController::TimeSliderCB()
{
	SetCurrentTime_ms((AniTime_ms)(time_slider_ms_->value()), false);
}



void
AnimationTimeController::PlayToggle()
{
	if ( flag_play_ )
	{
		flag_play_ = false;
		play_btn_->label("@>");
		NotifyPlayStopped();
		Fl::remove_idle(StaticIdleCB, this);
	}
	else
	{
		flag_play_ = true;
		play_btn_->label("||");
		//Fl::add_timeout(1./play_fps_, StaticTimerCB, this);
		g_acc_idle_pre_time = std::chrono::steady_clock::now();
		Fl::add_idle(StaticIdleCB, this);
		NotifyPlayStarted();
	}
}


void
AnimationTimeController::StaticIdleCB(void *d)
{
	((AnimationTimeController*)d)->IdleCB();
}

void
AnimationTimeController::IdleCB()
{

	
	
	if (flag_play_)
	{
		auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - g_acc_idle_pre_time);
		if (elapsed > g_acc_time_interval)
		{
			g_acc_idle_pre_time = std::chrono::steady_clock::now();
			g_acc_cur_time += elapsed;
		}
		else
		{
			return;
		}


		if (g_acc_cur_time >= g_acc_final_time)
		{
			switch (play_mode_)
			{
			case ONE_TIME:
				PlayToggle();
				break;
			case ONE_TIME_AND_REWIND:
				SetCurrentTime_ms(0);
				PlayToggle();
				break;
			case REPEAT:
				SetCurrentTime_ms(0);
				break;
			case TO_INFINITE:
				SetCurrentTime_ms((AniTime_ms)g_acc_cur_time.count());
				break;
			};

		}
		else
		{
			SetCurrentTime_ms((AniTime_ms)g_acc_cur_time.count());
		}
	}
}

};