
#pragma once


#include "BaseLib/FLTKU/Animation.h"

namespace mg
{

class AnimationBrowserListener
{
public:
	virtual void BrowserSelectionsChanged(const AnimationList &selections)=0;
	virtual void BrowserItemsChanged(const AnimationList &items)=0;
};

}
