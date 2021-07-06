
#pragma once
#include <vector>
#include <string>
#include "FL/Fl_Widget.H"
#include "BaseLib/FLTKU/Animation.h"

namespace mg
{

class AnimationController
{
public:
	AnimationController();

	virtual void RegisterAnimation(mg::Animation *ani);
	virtual bool IsRegisteredAnimation(mg::Animation *ani)  const;

	virtual void UnbindAllAnimation();
	virtual void BindAnimation(mg::Animation *ani);

	static void StaticHandle(Fl_Widget* w, void *d);
	virtual void Handle(Fl_Widget* w);
	virtual void Handle(mg::Animation *ani, Fl_Widget* w);
	virtual int HandleFLShortcutEvent(int fl_key);
	virtual int HandleFLShortcutEvent(mg::Animation *ani, int fl_key);

	virtual int num_widgets() const { return (int)widgets_.size(); }
	virtual Fl_Widget* widget(int i) const { return widgets_[i]; }
	virtual Fl_Widget* widget_by_label(std::string label) const;

	virtual inline int default_widget_h() const { return 20; }
	virtual inline int default_widget_w() const { return 100; }


	virtual Fl_Widget* AddFlCheckButton(const char* label, bool value, int shortcut_key);
	virtual Fl_Widget* AddFlCheckButton(const char* label, bool value=false);
	virtual Fl_Widget* AddFlButton(const char* label, int shortcut_key);
	virtual Fl_Widget* AddFlButton(const char* label);
	virtual Fl_Widget* AddFlBox(const char* label);

protected:
	std::vector<Fl_Widget*> widgets_;
	std::vector<mg::Animation*> registered_animations_;
	std::vector<mg::Animation*> binded_animations_;
};

}
