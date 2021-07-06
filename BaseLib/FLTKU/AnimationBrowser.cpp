

#include "BaseLib/FLTKU/AnimationBrowser.h"
#include "BaseLib/FLTKU/AnimationViewer.h"
#include <sstream>

namespace mg
{

AnimationBrowser::AnimationBrowser(int x, int y, int w, int h, const char *s) : Fl_Browser(x, y, w, h, s)
{
	type(3);
	box(FL_SHADOW_BOX);
	callback((Fl_Callback*)StaticBrowserCB, (void*)(this));

	end();
}


void
AnimationBrowser::NotifySelectionsChanged()
{
	AnimationList ani_list;
	for ( int i=0; i<this->size(); i++ )
	{
		if ( this->selected(i+1) )
		{
			Animation *a = (Animation*)this->data(i+1);
			ani_list.push_back(a);
		}
	}

	for ( int i=0; i<(int)ani_browser_listeners_.size(); i++ )
	{
		ani_browser_listeners_[i]->BrowserSelectionsChanged(ani_list);
	}

}


void
AnimationBrowser::NotifyItemsChanged()
{
	AnimationList ani_list;
	for ( int i=0; i<this->size(); i++ )
	{
		Animation *a = (Animation*)this->data(i+1);
		ani_list.push_back(a);
	}

	for ( int i=0; i<(int)ani_browser_listeners_.size(); i++ )
	{
		ani_browser_listeners_[i]->BrowserItemsChanged(ani_list);
	}
}


void
AnimationBrowser::AddAnimation(Animation *ani)
{
	this->add(ani->name().c_str(), ani);

	NotifyItemsChanged();
}

void
AnimationBrowser::RemoveAniBrowserListener(AnimationBrowserListener* a)
{
	for ( unsigned int i=0; i<ani_browser_listeners_.size(); i++ )
	{
		if ( ani_browser_listeners_[i] == a )
		{
			ani_browser_listeners_.erase( ani_browser_listeners_.begin() + i );
			break;
		}
	}
}

void
AnimationBrowser::SelectAni(std::string ani_name)
{
	for ( int i=0; i<this->size(); i++ )
	{
		if ( ani_name.compare( this->text(i+1) ) == 0 )
		{
			SelectAni(i);
			return;
		}
	}
}

void
AnimationBrowser::StaticBrowserCB(Fl_Widget *w, void *d)
{
	((AnimationBrowser*)d)->BrowserCB();
}

void
AnimationBrowser::BrowserCB()
{
	NotifySelectionsChanged();
}


};



