

#include "BaseLib/FLTKU/FL_GLEW_Window.h"
#include "GL/glew.h"
#include <iostream>
#include "FL/Fl.H"
#include "FL/x.h"


namespace mg
{

static void *g_gl_context_ = 0;
static mg::GL_Renderer *g_renderer_ = 0;

/**
Fl_Glew_Init_Window class is used only for creating the gl_context
that will be used for all windows opend in this application.

To create a complete gl_context, information of a window context must be provided. 
When you create an instance of Fl_Glew_Init_Window and open it,
a window context and a gl context are created. And the pointer to the
gl context will be assigned to the global variable g_gl_context_.

The window of Fl_Glew_Init_window can be closed immediately after creating 
the gl_context successfully. 

InitFlGlew() function includes these procedures.
*/
class Fl_Glew_Init_Window : public Fl_Gl_Window
{
public:
	Fl_Glew_Init_Window(int w, int h, const char *s=0) : Fl_Gl_Window(w, h, s)
	{
	}

	virtual void draw()
	{
		if ( !context_valid() )
		{
			g_gl_context_ = context();
		}
	}

	/**
	Override hide() to avoid destroying gl context.
	*/
	virtual void hide() override { Fl_Window::hide(); }

};


/**
This function creates a gl context and an instance of GL_Renderer
that will be used globally for this application.

All windows of Fl_Glew_Window or its children will have this gl_context
and GL_Renderer.

mg::InitBaseLib() must be called before InitFlGlew();
*/
void InitFlGlew()
{
	Fl_Glew_Init_Window *tmp = new Fl_Glew_Init_Window(10, 10);
	tmp->end();
	tmp->show();

	// Fl::check() executes Fl::run() one times,
	// so that "Fl_Glew_Init_Window *tmp" window completely is completely created and gl context so.
	while ( g_gl_context_ == 0 )
	{
		Fl::check();
	}

	/**
	*/
	if (  glewInit() != GLEW_OK )
	{
		std::cerr << "Unable to initialize GLEW ... exiting" << std::endl;
		exit(EXIT_FAILURE);
	}
	else
	{
		std::cout << "GLEW OK\n";
	}

	g_renderer_ = new mg::GL_Renderer;

	Fl_Glew_Window::renderer_ = g_renderer_;

	tmp->hide();
}


void DestroyFlGlew()
{
	if ( g_gl_context_ != 0 )
	{
#ifdef WIN32
		wglDeleteContext((HGLRC)g_gl_context_);
#else
	std::cerr << "need to call a function similar to wglDeleteContext()" << std::endl;
	exit(EXIT_FAILURE);
#endif
	}

	g_gl_context_ = 0;

	if ( g_renderer_ != 0 )
		delete g_renderer_;
	g_renderer_= 0;
}

mg::GL_Renderer *GetDefaultRenderer()
{
	return g_renderer_;
}







mg::GL_Renderer * Fl_Glew_Window::renderer_ = 0;


Fl_Glew_Window::Fl_Glew_Window(int w, int h, const char *s) : Fl_Gl_Window(w, h, s)
{
}

Fl_Glew_Window::Fl_Glew_Window(int x, int y, int w, int h, const char *s) : Fl_Gl_Window(x, y, w, h, s)
{
}

Fl_Glew_Window::~Fl_Glew_Window()
{
}


void 
Fl_Glew_Window::resize(int x, int y, int w, int h)
{
	Fl_Gl_Window::resize(x, y, w, h);
	renderer_->SetViewport(0, 0, w, h);
}

void
Fl_Glew_Window::draw()
{
	if ( g_gl_context_ != 0 )
	{
		// Replace the gl context.
#ifdef WIN32
		wglMakeCurrent((HDC)0, (HGLRC)0);
		wglMakeCurrent((HDC)(GetDC(fl_xid(this))), (HGLRC)g_gl_context_);
#else
		// need to call some function similar to wglMakeCurrent
		std::cerr << "Noneed to call some function similar to wglMakeCurrent ... exiting" << std::endl 
			<< "[ Fl_Glew_Window::draw() ]" << std::endl;
#endif
	}
	else
	{
		std::cerr << "No profer gl context exists. InitFlGlew() is may not called ... exiting" << std::endl 
			<< "[ Fl_Glew_Window::draw() ]" << std::endl;
		exit(EXIT_FAILURE);
	}
	

	Draw();
}


void
Fl_Glew_Window::DrawHeader()
{
	renderer_->ClearGLBuffers();
}

void
Fl_Glew_Window::Draw()
{
	DrawHeader();
	renderer_->UpdateProjectionViewMatrix();
	renderer_->SetModelIdentity();
}





std::pair<int, int>
Fl_Glew_Window::WinCoordToViewportCoord(int win_x, int win_y)
{

	int win_w = w();
	int win_h = h();
	
	double x = (double)win_x;
	double y = (double)(win_h-win_y);

	std::pair<int, int> out;
	out.first = (int) x;
	out.second = (int) y;

	return out;
}

std::pair<int, int>
Fl_Glew_Window::ViewportCoordToWinCoord(int vp_x, int vp_y)
{
	int win_w = w();
	int win_h = h();
	
	double x = (double)vp_x;
	double y = (double)(win_h-vp_y);

	std::pair<int, int> out;
	out.first = (int) x;
	out.second = (int) y;

	return out;
}

bool 
Fl_Glew_Window::UnProjectWinP(int win_x, int win_y, double z, cml::vector3d &out_p)
{
	
	GLint viewport[4];
	glGetIntegerv(GL_VIEWPORT,viewport);

	cml::Camera *camera = renderer_->camera();
	double *mv = camera->GetGLViewMatrix().data();
	double *p = camera->GetGLProjectionMatrix().data();

	std::pair<int, int> vp_p = WinCoordToViewportCoord(win_x, win_y);
	gluUnProject(vp_p.first, vp_p.second, z
				, mv, p, viewport
				, &out_p[0], &out_p[1], &out_p[2]);


	return true;
}


bool 
Fl_Glew_Window::UnProjectWinP(int win_x, int win_y, cml::vector3d &out_p)
{
	//glFlush();
	GLint viewport[4];
	glGetIntegerv(GL_VIEWPORT,viewport);
	std::pair<int, int> vp_p = WinCoordToViewportCoord(win_x, win_y);

	// Get the Z coordinate for the pixel location
	GLuint lDepth;
	glReadPixels( vp_p.first, vp_p.second, 
		1, 1, GL_DEPTH_COMPONENT, GL_UNSIGNED_INT, 
		&lDepth );


	if ( lDepth == UINT_MAX ) return false;

	int zdepth;
	double lDistTermInv;
	glGetIntegerv( GL_DEPTH_BITS, &zdepth );
	switch ( zdepth )
	{
	case 16 :
		lDistTermInv = 1.5259018967e-5; // 65535
		break;
	default :
		lDistTermInv = 2.32830643708e-10; // 4294967295
		break;
	}

	double lDistance = lDepth * lDistTermInv;

	cml::Camera *camera = renderer_->camera();
	double *mv = camera->GetGLViewMatrix().data();
	double *p = camera->GetGLProjectionMatrix().data();

	gluUnProject( vp_p.first, vp_p.second, 
		lDistance, mv, p, viewport,
		&out_p[ 0 ], &out_p[ 1 ], &out_p[ 2 ] );

	return true;
}





int
Fl_Glew_Window::handle(int event)
{

	cml::Camera *camera = renderer_->camera();
	if ( !camera ) return Fl_Gl_Window::handle(event);

	////////////////////////////////////////////////////
	// camera handling
	static double oldX, oldY;
	static double x, y;
	static bool flag_button1 = false;
	static bool flag_button2 = false;
	static bool flag_button12 = false;
	int winW = w();
	int winH = h();
	static bool flag_rotation_pivot = false;
	static cml::vector3d rotation_pivot;

	if ( event == FL_ENTER || event == FL_FOCUS )
	{
		resize(this->x(), this->y(), w(), h());
		redraw();
		return 1;
	}

	else if ( event == FL_PUSH )
	{
		cml::vector3d pivot;
		pivot = cml::vector3d(0, 0, 0);
		camera->setPivot(pivot);

		x = (double)Fl::event_x() / winW;
		y = 1 - (double)Fl::event_y() / winH;

		if ( Fl::event_button() == FL_LEFT_MOUSE )
		{
			if ( flag_button2 == true )
			{
				flag_button1 = false;
				flag_button2 = false;
				flag_button12 = true;
			}
			else
			{
				flag_button1 = true;
				if (UnProjectWinP(winW / 2, winH / 2, rotation_pivot))
				{
					flag_rotation_pivot = true;
				}
				else
				{
					flag_rotation_pivot = false;
				}
			}flag_rotation_pivot = false;

			
			redraw();
			return 1;

		}
		else if ( Fl::event_button() == FL_RIGHT_MOUSE )
		{
			if ( flag_button1 == true )
			{
				flag_button1 = false;
				flag_button2 = false;
				flag_button12 = true;
			}
			else
				flag_button2 = true;

			redraw();
			return 1;

		}
	}
	else if ( event == FL_DRAG )
	{
		oldX = x;
		oldY = y;

		x = (double)Fl::event_x() / winW;
		y = 1 - (double)Fl::event_y() / winH;

		if ( flag_button1 )
		{
			if ( flag_rotation_pivot )
				camera->inputMouse(cml::Camera::IN_ROTATION_Y_UP, oldX, oldY, x, y);//, rotation_pivot);
			else
				camera->inputMouse(cml::Camera::IN_ROTATION_Y_UP, oldX, oldY, x, y);
			redraw();
			return 1;
		}
		else if ( flag_button2 )
		{
			camera->inputMouse(cml::Camera::IN_TRANS, oldX, oldY, x, y, 2.0);

			redraw();
			return 1;
		}
		else if ( flag_button12 )
		{
			camera->inputMouse(cml::Camera::IN_ZOOM, oldX, oldY, x, y);

			redraw();
			return 1;
		}
	}
	else if ( event == FL_RELEASE )
	{
		if ( Fl::event_button() == FL_LEFT_MOUSE )
		{
			flag_button1 = false;
			flag_button12 = false;
			return 1;
		}
		else if ( Fl::event_button() == FL_RIGHT_MOUSE )
		{
			flag_button2 = false;
			flag_button12 = false;
			return 1;
		}

	}
	else if ( event == FL_MOUSEWHEEL )
	{
		cml::vector3d pivot(0, 0, 0);
		if ( false && !UnProjectWinP(Fl::event_x(), Fl::event_y(), pivot) )
		{
			pivot = cml::vector3d(0, 0, 0);
		}
		
		camera->setPivot(pivot);
		

		if ( Fl::event_ctrl() )
			camera->inputMouse(cml::Camera::IN_FOV, 0, Fl::event_dy());
		if ( Fl::event_alt() )
			camera->inputMouse(cml::Camera::IN_ZOOM, 0, -1 * Fl::event_dy());
		else
			camera->inputMouse(cml::Camera::IN_TRANS_Z, 0, Fl::event_dy());
		redraw();
		return 1;
	}


	return Fl_Gl_Window::handle(event);
}


};