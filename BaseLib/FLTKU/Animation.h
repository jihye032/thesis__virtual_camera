

#pragma once



#include "BaseLib/FLTKU/AnimationTimeListener.h"
#include "BaseLib/FLTKU/AnimationListener.h"
#include "BaseLib/CmlExt/CmlExt.h"
#include "BaseLib/GL4U/GL_Renderer.h"
#include <string>
#include <vector>

namespace mg
{

class AnimationViewer;


class Animation : public AnimationTimeListener
{
public:
	Animation(GL_Renderer *r)
	{
		renderer_ = r;
		flag_shadow_phase_ = false;
		fps_ = 40;
		cur_time_ms_ = 0;
		final_ani_time_ms_ = 1000;
		type_ = "Animation";
		name_ = "noname_";
		global_transf_.identity();

	}

	Animation()
	{
		renderer_ = GetDefaultRenderer();
		flag_shadow_phase_ = false;
		fps_ = 40;
		cur_time_ms_ = 0;
		final_ani_time_ms_ = 1000;
		type_ = "Animation";
		name_ = "noname_";
		global_transf_.identity();
	}

	virtual ~Animation()
	{
	}
	


	virtual void Draw(AnimationViewer *ani_viewer) = 0;
	virtual void Draw(AnimationViewer *ani_viewer, AniTime_ms from, AniTime_ms to, AniTime_ms step);
	virtual void DrawForPicking(AnimationViewer *ani_viewer) 
	{
		////// Example
		/*glInitNames();

		glPushName(1);
		drawBody();
		glPopName();

		glPushName(2);
		drawHead();
		drawEyes();
		glPopName();*/

		/*
		renderer_->SetPickName(1);
		drawBody();
		glPopName();

		renderer_->SetPickName(2);
		drawHead();
		*/
	}


	/// AnimationFrameListener
	virtual void TimeChanged(AniTime_ms t_ms) override;



	virtual void AddAnimationListener(AnimationListener* al) { ani_listeners_.push_back(al); }

	virtual void NotifyAnimationUpdated();
	virtual void AddAniViewer(AnimationViewer *a);
	virtual void RemoveAniViewer(AnimationViewer *a);
	virtual int HandleFLTK(AnimationViewer *w, int event) { return 0; }

	virtual cml::matrix44d global_transf() const { return global_transf_; }

	// Times
	virtual void SetCurrentTime(AniTime_ms t_ms);
	virtual int GetCurFrameId() const { return AniTimeMs_to_FrameId(cur_time_ms_); }
	virtual double GetCurrentTime_s() const { return 0.001*cur_time_ms_; }
	virtual AniTime_ms cur_time_ms() const { return cur_time_ms_; }
	virtual void final_ani_time_ms(AniTime_ms t) { final_ani_time_ms_ = t; }
	virtual AniTime_ms final_ani_time_ms() const { return final_ani_time_ms_; }
	virtual AniTime_ms GetTotalAnimationTimeLen_ms() const { return final_ani_time_ms_ +1; }


	// Frames
	virtual void fps(int f) { fps_ = f; }
	virtual double fps() const { return fps_; }
	virtual int CountTotalFrames() const { return (int)(GetTotalAnimationTimeLen_ms()*0.001*fps_); }

	virtual void name(const std::string &n) { name_ = n; }
	virtual std::string name() const { return name_; }

	const std::string& type() const { return type_; }


	const GL_Renderer* renderer() const { return renderer_; }
	
	
protected:
	inline int AniTimeMs_to_FrameId(AniTime_ms t) const { return (int)(t*0.001*fps_); }


protected:
	GL_Renderer *renderer_;

	double fps_;
	AniTime_ms cur_time_ms_;
	AniTime_ms final_ani_time_ms_;

	bool flag_shadow_phase_;

	std::string name_;

	std::vector<AnimationViewer*> ani_viewers_;
	std::vector<AnimationListener*> ani_listeners_;

	cml::matrix44d_c global_transf_;

	std::string type_;
};



class AnimationList : public std::vector<Animation*>
{
public:
	int GetMaxFrameNum() const
	{
		int max = 0;
		for ( int i=0; i<(int)size(); i++ )
		{
			int tmp = (*this)[i]->CountTotalFrames();
			if ( max < tmp ) max = tmp;
		}

		return max;
	};

	AniTime_ms GetMaxFinalAniTime_ms() const
	{
		AniTime_ms max = 0;
		for (int i = 0; i<(int)size(); i++)
		{
			AniTime_ms tmp = (*this)[i]->final_ani_time_ms();
			if (max < tmp) max = tmp;
		}

		return max;
	};

	int CountAnimations() const
	{ 
		return (int)size(); 
	}
};

};