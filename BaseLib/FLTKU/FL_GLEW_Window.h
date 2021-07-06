#pragma once

#include "GL/glew.h"
#include "FL/Fl_Gl_Window.H"
#include "BaseLib/CmlExt/CmlCamera.h"
#include "BaseLib/GL4U/GL_Renderer.h"

/**
 Reference: http://glew.sourceforge.net/advanced.html
*/


namespace mg
{

/** 
Fl_Glew_Window class is a window for rendering by using Glew library,
which enables to use OpenGL 3.3 and later version.

Before creating and displaying a Fl_Glew_Window window, you must initialize
glew by calling InitGlGlew() instead of initGlew().
You must not call initGlew(), initGlew() will be called in InitFlGlew() fuction.

renderer_ is the pointer to the default Gl_Renderer instance, which is a
global variable shared for all Fl_Glew_Window and its childern classes.
*/

class Fl_Glew_Window : public Fl_Gl_Window
{
	friend void InitFlGlew();

public:
	Fl_Glew_Window(int w, int h, const char *s=0);
	Fl_Glew_Window(int x, int y, int w, int h, const char *s=0);
	virtual ~Fl_Glew_Window();
	
	
	virtual void resize(int x, int y, int w, int h);

	std::pair<int, int> WinCoordToViewportCoord(int win_x, int win_y);
	std::pair<int, int> ViewportCoordToWinCoord(int vp_x, int vp_y);
	bool Fl_Glew_Window::UnProjectWinP(int win_x, int win_y, double z, cml::vector3d &out_p);
	bool Fl_Glew_Window::UnProjectWinP(int win_x, int win_y, cml::vector3d &out_p);

	virtual int handle(int event);

	// const cml::Camera& default_camera() const { return default_camera_; }

private:
	/**
	draw() is inherited from Fl_Gl_Window.
	In draw(), the gl context is replaced by the global one (g_gl_context_),
	which is shared for all Fl_Glew_Window in this application.
	Because this process isis very important,
	we keep the inside of draw() function as simiple as possible,
	to avoid any possibility of a mess-up.
	General rendering code should be written in the Draw() function.
	*/
	virtual void draw() final;


protected:
	virtual void DrawHeader();
	virtual void Draw();

protected:
	// bool flag_default_camera_on_;
	// cml::Camera default_camera_;

	static mg::GL_Renderer *renderer_;
};

/**
This function creates a gl context and an instance of GL_Renderer
that will be used globally for this application.

All windows of Fl_Glew_Window or its children will have this gl_context
and GL_Renderer.

mg::InitBaseLib() must be called before InitFlGlew();
*/
void InitFlGlew();
void DestroyFlGlew();

};