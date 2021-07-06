




#include "BaseLib/FLTKU/Animation.h"
#include <algorithm>
#include "BaseLib/FLTKU/AnimationViewer.h"
#include "BaseLib/GL4U/GL_Renderer.h"

namespace mg
{




void
Animation::TimeChanged(AniTime_ms t_ms)
{
	SetCurrentTime(t_ms);
}

void 
Animation::SetCurrentTime(AniTime_ms t_ms)
{ 
	AniTime_ms pre_time_ms = cur_time_ms_;

	if (t_ms<0 ) cur_time_ms_ =0;
	else if (t_ms >= final_ani_time_ms_) cur_time_ms_ = final_ani_time_ms_;
	else cur_time_ms_ = t_ms;

	if ( AniTimeMs_to_FrameId(pre_time_ms) != AniTimeMs_to_FrameId(cur_time_ms_) )
		NotifyAnimationUpdated();
}

void
Animation::NotifyAnimationUpdated()
{
	for ( int i=0; i<(int)ani_viewers_.size(); i++ )
	{
		ani_viewers_[i]->ContentsUpdated();
	}

	for ( int i=0; i<(int)ani_listeners_.size(); i++ )
	{
		ani_listeners_[i]->AnimationChanged(this);
	}
}

void
Animation::AddAniViewer(AnimationViewer *v)
{
	if ( std::find(ani_viewers_.begin(), ani_viewers_.end(), v) == ani_viewers_.end() )
	{
		ani_viewers_.push_back(v);
	}
}

void
Animation::RemoveAniViewer(AnimationViewer *v)
{
	std::vector<AnimationViewer*>::iterator iter = std::find(ani_viewers_.begin(), ani_viewers_.end(), v);

	if ( iter != ani_viewers_.end() )
	{
		ani_viewers_.erase(iter);
	}
}

void
Animation::Draw(AnimationViewer *ani_viewer, AniTime_ms from, AniTime_ms to, AniTime_ms step)
{
	
}


};







