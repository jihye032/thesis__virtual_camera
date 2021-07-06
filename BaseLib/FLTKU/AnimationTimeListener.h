
#pragma once

namespace mg
{

typedef unsigned int AniTime_ms;

class AnimationTimeListener
{
public:
	virtual void TimeChanged(AniTime_ms t_ms)=0;
	virtual void TimeReset()
	{
		TimeChanged(0);
	}

	virtual void PlayStarted(int f){};
	virtual void PlayStopped(int f){};
};

};

