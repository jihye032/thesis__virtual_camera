
#pragma once

namespace mg
{

class Animation;

class AnimationListener
{
public:
	virtual void AnimationChanged(const Animation *ani)=0;
};


};