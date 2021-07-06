
#pragma once

#include <FL/Fl.H>
#include <FL/Fl_Window.H>
#include <FL/Fl_Browser.H>


#include "BaseLib/FLTKU/AnimationBrowserListener.h"

namespace mg
{

class Animation;


class AnimationBrowser : public Fl_Browser
{
public:
	AnimationBrowser(int x, int y, int w, int h, const char *s=0);

	void AddAnimation(Animation* ani);

	void AddAniBrowserListener(AnimationBrowserListener* a) { ani_browser_listeners_.push_back(a); }
	void RemoveAniBrowserListener(AnimationBrowserListener* a);

	void SelectAni(int i) { this->select(i+1); BrowserCB(); }
	void SelectAni(std::string ani_name); 

protected:
	static void StaticBrowserCB(Fl_Widget *w, void *d);
	void BrowserCB();

	void NotifySelectionsChanged();
	void NotifyItemsChanged();

protected:
	std::vector<AnimationBrowserListener*> ani_browser_listeners_;
};

};