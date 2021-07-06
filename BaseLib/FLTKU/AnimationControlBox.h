
#pragma once

#include "FL/Fl_Window.H"
#include "FL/Fl_Scroll.H"
#include "FL/Fl_Pack.H"
#include "BaseLib/FLTKU/AnimationBrowserListener.h"
#include "BaseLib/FLTKU/AnimationController.h"
#include <map>
#include <vector>

namespace mg
{

class AnimationControlBox : public Fl_Window, public AnimationBrowserListener
{
public:
	AnimationControlBox(int x, int y, int w, int h, const char *s=0);
	AnimationControlBox(int w, int h, const char *s=0);

	void AddController(AnimationController *c);

	AnimationController* FindControllerFor(Animation* ani);
	void SwitchDisplayedController(AnimationController *c);
	void Undisplay();

	virtual int handle(int event);
	 
protected:

	void Init();

	// AnimationBrowserListener
	virtual void BrowserItemsChanged(const AnimationList &items){};
	virtual void BrowserSelectionsChanged(const AnimationList &selections);
	
protected:
	Fl_Scroll *scroll_;
	Fl_Pack *pack_;
	std::vector<AnimationController*> controllers_;
	AnimationController *displayed_controller_;
};


};
