
#include "BaseLib/FLTKU/AnimationController.h"
#include "FL/Fl_Button.h"
#include "FL/Fl_Check_Button.h"
#include "FL/Fl_Box.h"
#include <algorithm>

namespace mg
{

AnimationController::AnimationController()
{
}

void
AnimationController::RegisterAnimation(mg::Animation *ani)
{
	registered_animations_.push_back(ani);
}

bool
AnimationController::IsRegisteredAnimation(mg::Animation *ani) const
{
	if ( std::find(registered_animations_.begin(), registered_animations_.end(), ani)
		!= registered_animations_.end() ) 
		return true;
	else
		return false;
}

void
AnimationController::UnbindAllAnimation()
{
	binded_animations_.clear();
}

void
AnimationController::BindAnimation(mg::Animation *ani)
{
	if ( !IsRegisteredAnimation(ani) ) return;

	if ( std::find(binded_animations_.begin(), binded_animations_.end(), ani)
		== binded_animations_.end() )
	{
		binded_animations_.push_back(ani);
	}
}

void
AnimationController::StaticHandle(Fl_Widget* w, void *d)
{
	((AnimationController*)d)->Handle(w);
}

void
AnimationController::Handle(Fl_Widget *w)
{
	for ( unsigned int i=0; i<binded_animations_.size(); i++ )
	{
		Handle(binded_animations_[i], w);
	}
}

void
AnimationController::Handle(mg::Animation *ani, Fl_Widget *w)
{
	// Do Something.
}


int
AnimationController::HandleFLShortcutEvent(int fl_key)
{
	int result = 0;
	for ( unsigned int i=0; i<binded_animations_.size(); i++ )
	{
		int r = HandleFLShortcutEvent(binded_animations_[i], fl_key);

		if ( r != 0 ) result = r;
	}

	return result;
}

int
AnimationController::HandleFLShortcutEvent(mg::Animation *ani, int fl_key)
{
	

	return 0;
}

Fl_Widget*
AnimationController::widget_by_label(std::string label) const
{
	for ( unsigned int i=0; i<widgets_.size(); i++ )
	{
		if ( widgets_[i] != 0 
			&& widgets_[i]->label() != 0 
			&& label.compare(widgets_[i]->label()) == 0 )
		{
			return widgets_[i];
		}
	}

	return 0;
}


Fl_Widget*
AnimationController::AddFlCheckButton(const char* label, bool value, int shortcut_key)
{
	Fl_Check_Button *c = (Fl_Check_Button*)AddFlCheckButton(label, value);
	c->shortcut(shortcut_key);
	return c;
}

Fl_Widget*
AnimationController::AddFlCheckButton(const char* label, bool value)
{
	Fl_Check_Button *c = new Fl_Check_Button(0, 0, default_widget_w(), default_widget_h(), label);
	c->value(value);
	c->callback(StaticHandle, this);
	widgets_.push_back(c);
	return c;
}

Fl_Widget*
AnimationController::AddFlButton(const char* label, int shortcut_key)
{
	Fl_Button *b = (Fl_Button*)AddFlButton(label);
	b->shortcut(shortcut_key);
	return b;
}

Fl_Widget*
AnimationController::AddFlButton(const char* label)
{
	Fl_Button *b = new Fl_Button(0, 0, default_widget_w(), default_widget_h(), label);
	b->callback(StaticHandle, this);
	widgets_.push_back(b);
	return b;
}

Fl_Widget*
AnimationController::AddFlBox(const char* label)
{
	Fl_Box *b = new Fl_Box(0, 0, default_widget_w(), default_widget_h(), label);
	b->callback(StaticHandle, this);
	widgets_.push_back(b);
	return b;
}

};