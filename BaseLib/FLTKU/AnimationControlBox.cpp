
#include "BaseLib/FLTKU/AnimationControlBox.h"
#include "FL/Fl.H"

namespace mg
{


AnimationControlBox::AnimationControlBox(int x, int y, int w, int h, const char *s) : Fl_Window(x, y, w, h, s)
{
	Init();
}

AnimationControlBox::AnimationControlBox(int w, int h, const char *s) : Fl_Window(w, h, s)
{
	Init();
}

void
AnimationControlBox::Init()
{
	displayed_controller_ = 0;
	scroll_ = new Fl_Scroll(1, 1, w()-2, h()-2);
	scroll_->type(Fl_Scroll::VERTICAL);
	scroll_->color(FL_WHITE);

	pack_ = new Fl_Pack(1, 1, w()-2 - scroll_->scrollbar.w(), h()-2);
	pack_->type(Fl_Pack::VERTICAL);
	pack_->color(FL_WHITE);
	pack_->spacing(1);
	pack_->end();
	scroll_->resizable(pack_);
	scroll_->end();

	this->resizable(scroll_);
}



void 
AnimationControlBox::BrowserSelectionsChanged(const AnimationList &selections)
{
	// Find the controller for the first animation in selections.
	AnimationController *controller = 0;
	if ( !selections.empty() )
		controller = FindControllerFor( selections.front() );


	// If no matched controller for the selected animation.
	if ( controller == 0 )
	{
		Undisplay();
		return;
	}


	// Bind the selected animation with the controller.
	controller->UnbindAllAnimation();
	for ( unsigned int i=0; i<selections.size(); i++ )
	{
		controller->BindAnimation(selections[i]);
	}


	// Display the Controller.
	SwitchDisplayedController(controller);
}


void
AnimationControlBox::AddController(AnimationController *c)
{
	controllers_.push_back(c);
}

AnimationController* 
AnimationControlBox::FindControllerFor(Animation* ani)
{
	for ( unsigned int i=0; i<controllers_.size(); i++ )
	{
		if ( controllers_[i]->IsRegisteredAnimation(ani) ) return controllers_[i];
	}

	return 0;
}

void
AnimationControlBox::Undisplay()
{
	while ( pack_->children() > 0 )
	{
		pack_->remove(0);
	}
	displayed_controller_ = 0;
	redraw();
}

void
AnimationControlBox::SwitchDisplayedController(AnimationController *c)
{
	Undisplay();
	
	//int widget_h = 20;
	pack_->size(pack_->w(), 10);//widget_h*c->num_widgets());

	for ( int i=0; i<c->num_widgets(); i++ )
	{
		Fl_Widget *w = c->widget(i);
		//w->size(w->w(), widget_h);
		pack_->add(w);
	}

	displayed_controller_ = c;

	redraw();
}

int
AnimationControlBox::handle(int event)
{
	int result = 0;
	if ( event == FL_SHORTCUT && displayed_controller_ != 0 )
	{
		result = displayed_controller_->HandleFLShortcutEvent(Fl::event_key());
	}

	if ( result == 0 )
		return Fl_Window::handle(event);
	else
		return result;
}




};
