
#include "BaseLib/FLTKU/AnimationViewer.h"
//#include "BaseLib/GLUU/gluu.h"
#include "FL/Fl.H"
#include "FL/x.H"
#ifdef WIN32
#include <Commdlg.h>
#include <direct.h>
#else
#include <sys/stat.h>
#include <errno.h>
#endif
#include <sstream>
#include <time.h>
#include <iomanip>
#include <wchar.h>

namespace mg
{

AnimationViewer::AnimationViewer(int x, int y, int w, int h, const char* s) : Fl_Glew_Window(x, y, w, h, s)
{
	// display
	flag_ghost_frames_ = false;
	flag_ground_ = true;
	flag_line_character_ = false;

	// alignment of charaters
	flag_align_in_grid_ = false;
	grid_max_col_ = 5;
	grid_gap_ = 160;

	flag_auto_frame_capture_ = false;

	MakeAndSetBaseOutDirectoryPath("");
	SetOutDirForCapturedImages("");
	SetCaptureImageFilePrefix("image_");

	end();

	InitGlew();
}

AnimationViewer::AnimationViewer(int w, int h, const char *s) : Fl_Glew_Window(w, h, s)
{
	// display
	flag_ghost_frames_ = false;
	flag_ground_ = true;
	flag_line_character_ = false;

	// alignment of charaters
	flag_align_in_grid_ = false;
	grid_max_col_ = 5;
	grid_gap_ = 60;

	flag_auto_frame_capture_ = false;

	MakeAndSetBaseOutDirectoryPath("");
	SetOutDirForCapturedImages("");
	SetCaptureImageFilePrefix("image_");

	end();

	InitGlew();
}

AnimationViewer::~AnimationViewer()
{
	RemoveAllAnimations();

}



////////////////////////////////////////////////////////////////////////////////////////////
// Animations


void
AnimationViewer::AddAnimation(Animation *ani)
{
	animations_.push_back(ani);
	ani->AddAniViewer(this);
}

void
AnimationViewer::SetAnimations(const AnimationList &ani_list)
{
	RemoveAllAnimations();

	for ( int i=0; i<(int)ani_list.size(); i++ )
	{
		animations_.push_back(ani_list[i]);
		animations_.back()->AddAniViewer(this);
	}

	redraw();
}


void
AnimationViewer::RemoveAllAnimations()
{
	for ( int i=0; i<(int)animations_.size(); i++ )
	{
		animations_[i]->RemoveAniViewer(this);
	}

	animations_.clear();
}


unsigned int 
AnimationViewer::Pick(int m_x, int m_y)
{
	cml::vector3d tmp;
	return Pick(m_x, m_y, tmp);
}

unsigned int 
AnimationViewer::Pick(int m_x, int m_y, cml::vector3d &out_point_on_surface)
{
	unsigned int picked_name = 0;

	glDisable(GL_MULTISAMPLE);
	renderer_->BeginPickingRender();
	
	DrawForPicking(m_x, m_y);
	
	
	picked_name = renderer_->Pick(m_x, m_y);
	//if ( picked_name !=0 ) 
	{
		if ( !renderer_->PickPositionOnRenderedSurface(m_x, m_y, out_point_on_surface) )
		{
			out_point_on_surface.set(0, 0, 0);
		}
	}

	renderer_->EndPickingRender();
	glEnable(GL_MULTISAMPLE);

	return picked_name;
}


////////////////////////////////////////////////////////////////////////////////////////////
// draw

void AnimationViewer::InitGlew()
{
	if ( renderer_ == 0 ) 
	{
		std::cerr << "renderer_ was not created correctly, AnimationViewer::InitGlew()" << std::endl;
		return;
	}

	renderer_->EnableLight(0, true);
	renderer_->SetLight(0, 0, cml::vector3f(0.f, 0.f, -1.f));
}



void AnimationViewer::Draw()
{
	Fl_Glew_Window::Draw();

	if ( flag_ground_ ) DrawGround();

	DrawContents();
}

void AnimationViewer::DrawForPicking(int m_x, int m_y)
{
	/*if ( !valid() )
	{
		InitGL();
	}
	
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();

	GLint viewport[4];
	glGetIntegerv(GL_VIEWPORT,viewport);
	gluPickMatrix(m_x,viewport[3]-m_y, 5,5,viewport);


	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


	camera_.glProjection();

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	camera_.glTranslform();

	DrawContentsForPicking();*/ 
	// ??

	cml::vector4f tmp_clear_color = renderer_->clear_color();
	renderer_->clear_color({0., 0., 0., 0});	// To make the background's picking-name be 0.
	renderer_->ClearGLBuffers();
	renderer_->UpdateProjectionViewMatrix();
	renderer_->SetModelIdentity();
	renderer_->clear_color(tmp_clear_color);

	DrawContentsForPicking();
}

void AnimationViewer::DrawContents()
{

	if ( flag_align_in_grid_ && grid_max_col_ > 0 )
	{
		int colNum = grid_max_col_;
		int rowNum = animations_.CountAnimations() / grid_max_col_ + 1;

		if ( animations_.CountAnimations() <= colNum )
		{
			colNum = animations_.CountAnimations();
			rowNum = 1;
		}
		

		double fx = grid_gap_ * (colNum/2);
		double fy = grid_gap_ * (rowNum/2);;
		for ( int i=0; i<animations_.CountAnimations(); i++ )
		{
			Animation *ani = animations_[i];

			if ( i % colNum == 0 && i > 0 )
			{
				fx = grid_gap_ * (colNum/2);
				fy -= grid_gap_;
			}


			// mg::mgluPush(GL_LIGHTING_BIT | GL_CURRENT_BIT);
			renderer_->PushModelMatrix();
			{
				renderer_->MultiModelMatrix(cml::MatrixTranslation(fx, 0.0, fy));

				cml::matrix44d t = ani->global_transf();
				// t = PlaneProject(t);
				
				t.inverse();
				renderer_->MultiModelMatrix(t);
				// mg::mgluRotateQ(t.getRotation().inverse());
				// mg::mgluTranslateV(-1*t.getTranslation());
				
				// mg::mgluColorMaterial(i);
				renderer_->SetColor(i);
				ani->Draw(this);
			}
			renderer_->PopModelMatrix();
			// mg::mgluPop();

			
			fx -= grid_gap_;
		}
	}
	else 
	{
		for ( int i=0; i<(int)animations_.CountAnimations(); i++ )
		{
			Animation *ani = animations_[i];

			/*
			mg::mgluPush(GL_LIGHTING_BIT | GL_CURRENT_BIT);
			
			mg::mgluColorMaterial(0.5, 0.5, 0.5);
			mg::mgluColorMaterial(i);
			*/

			renderer_->SetColor(i);
			ani->Draw(this);

			// mg::mgluPop();
		}
	}

}

void
AnimationViewer::DrawContentsForPicking()
{
	if ( flag_align_in_grid_ && grid_max_col_ > 0 )
	{
		int colNum = grid_max_col_;
		int rowNum = animations_.CountAnimations() / grid_max_col_ + 1;

		if ( animations_.CountAnimations() <= colNum )
		{
			colNum = animations_.CountAnimations();
			rowNum = 1;
		}
		

		double fx = grid_gap_ * (colNum/2);
		double fy = grid_gap_ * (rowNum/2);;
		for ( int i=0; i<animations_.CountAnimations(); i++ )
		{
			Animation *ani = animations_[i];

			if ( i % colNum == 0 && i > 0 )
			{
				fx = grid_gap_ * (colNum/2);
				fy -= grid_gap_;
			}


			// mg::mgluPush(GL_LIGHTING_BIT | GL_CURRENT_BIT);
			renderer_->PushModelMatrix();
			{
				renderer_->MultiModelMatrix(cml::MatrixTranslation(fx, 0.0, fy));

				cml::matrix44d t = ani->global_transf();
				// t = PlaneProject(t);

				t.inverse();
				renderer_->MultiModelMatrix(t);
				// mg::mgluRotateQ(t.getRotation().inverse());
				// mg::mgluTranslateV(-1*t.getTranslation());

				// mg::mgluColorMaterial(i);
				renderer_->SetColor(i);
				ani->DrawForPicking(this);
			}
			renderer_->PopModelMatrix();
			// mg::mgluPop();


			fx -= grid_gap_;
		}
	}
	else 
	{
		for ( int i=0; i<(int)animations_.CountAnimations(); i++ )
		{
			Animation *ani = animations_[i];

			/*
			mg::mgluPush(GL_LIGHTING_BIT | GL_CURRENT_BIT);

			mg::mgluColorMaterial(0.5, 0.5, 0.5);
			mg::mgluColorMaterial(i);
			*/

			renderer_->SetColor(i);
			ani->DrawForPicking(this);

			// mg::mgluPop();
		}
	}

}



void AnimationViewer::DrawGround()
{
	//renderer_->shader()->SetShadingMode(0);
	//renderer_->shader()->EnableSkinning(false);
	//renderer_->shader()->EnableTexture(false);
	glLineWidth(1);
	renderer_->Draw("ground");
}









////////////////////////////////////////////////////////////////////////////////////////////////////////
// event 
int
AnimationViewer::handle(int eve)
{
	if ( eve == FL_SHORTCUT )
	{
		char key = Fl::event_key();
		if ( cb_key_funcs_.find(key) != cb_key_funcs_.end() )
		{
			cb_key_funcs_[key](this, cb_key_funcs_params_[key]);
			return 1;
		}
	}

	int result = 0;
	for ( int i=0; i<animations_.CountAnimations(); i++ )
	{
		int tmp_result = animations_[i]->HandleFLTK(this, eve);
		if ( tmp_result != 0 )
		{
			result = tmp_result;
		}
	}

	if ( result != 0 ) return result;

	return Fl_Glew_Window::handle(eve);
}

void
AnimationViewer::SetCallBackKey(void (*cb)(AnimationViewer*, void*), char key, void *data)
{
	cb_key_funcs_[key] = cb;
	cb_key_funcs_params_[key] = data;
}







////////////////////////////////////////////////////////////////////////////////////////////////////////
// image capture 
#ifdef WIN32
BITMAPINFO 
AnimationViewer::GetBitmapInfo()
{
	HWND hWnd;
	RECT rect;
	BITMAPINFO binfo;

	make_current();
	hWnd = fl_xid(this);

	GetWindowRect( hWnd, &rect );
	int height = rect.bottom-rect.top;
	int width  = rect.right-rect.left;


	int ww = (width/8)*8, hh = (height/8)*8;
	int dx=(ww+3)&(0xfffffffc);
	int dy=(hh+3)&(0xfffffffc);

	binfo.bmiHeader.biSize			= 40;
	binfo.bmiHeader.biWidth 		= dx;
	binfo.bmiHeader.biHeight		= dy;
	binfo.bmiHeader.biPlanes		= 1;
	binfo.bmiHeader.biBitCount		= 24;
	binfo.bmiHeader.biCompression	= BI_RGB;
	binfo.bmiHeader.biSizeImage		= dy*(DWORD)((dx*3+3)&~3);
	binfo.bmiHeader.biClrUsed		= 0;
	binfo.bmiHeader.biClrImportant	= 0;

	return binfo;
}
#endif

int 
AnimationViewer::Capture( int ww, int hh, unsigned char* pPixelData )
{
	make_current();
	glReadPixels(0, 0, ww, hh, GL_BGR_EXT,GL_UNSIGNED_BYTE, pPixelData); 
	return 1;
}

#define DIB_HEADER_MARKER   ((WORD) ('M' << 8) | 'B')

void 
AnimationViewer::WriteBMP( const char* fn )
{
#ifdef WIN32
	make_current();
	char filename[1024];
	if( fn==NULL )
	{
		HWND hwnd = fl_xid(this);

		static	char fn[_MAX_PATH];
		static	char tl[_MAX_FNAME+_MAX_EXT];
		static	char title[_MAX_FNAME+_MAX_EXT];
		static	OPENFILENAME ofn =	{sizeof(OPENFILENAME),hwnd,0, "BMP File (*.BMP)\0*.bmp\0",
				0,0,1,fn,_MAX_PATH,tl,_MAX_FNAME+_MAX_EXT,0,title,0,0,0,0,0,0};

	
		if( GetSaveFileName( &ofn ) == FALSE ){	return;}
		if( ofn.lpstrFile[strlen(ofn.lpstrFile)-4]!='.' )
			strcat_s( ofn.lpstrFile, 4, ".BMP" );
		strcpy_s( filename, ofn.lpstrFile );
	}
	else
		strcpy_s( (char*)filename, strlen((char*)fn), (char*)fn );

	BITMAPINFO binfo = GetBitmapInfo();
	unsigned char* data = new unsigned char[binfo.bmiHeader.biSizeImage];
	redraw();
	Fl::wait(0.0);
	Capture( binfo.bmiHeader.biWidth, binfo.bmiHeader.biHeight, data );

	FILE* file;
	BITMAPFILEHEADER bmfHdr;

	fopen_s(&file, filename, "wb+" );

	bmfHdr.bfType = DIB_HEADER_MARKER;  // "BM"
	bmfHdr.bfSize=sizeof(BITMAPINFOHEADER)+binfo.bmiHeader.biSizeImage+sizeof(BITMAPFILEHEADER);

	bmfHdr.bfReserved1 = 0;
	bmfHdr.bfReserved2 = 0;
	bmfHdr.bfOffBits=(DWORD)sizeof(BITMAPFILEHEADER)+sizeof(BITMAPINFOHEADER);

	fwrite( (LPSTR)&bmfHdr, sizeof(BITMAPFILEHEADER),1,file);
	fwrite( &(binfo.bmiHeader), sizeof(BITMAPINFOHEADER), 1, file );
	fwrite( data, binfo.bmiHeader.biSizeImage, 1, file );

	fclose( file );
	delete data;

	return;
#endif
}

void
AnimationViewer::AutoFrameCapture(int num)
{
	std::stringstream file_s;
	file_s << base_out_dir_ << "/" << capture_out_dir_ << "/" << capture_image_file_prefix_ << std::setw(5) << std::setfill('0') << num << ".bmp";
	WriteBMP(file_s.str().c_str());
}

bool
AnimationViewer::MakeAndSetUniqueDirForCapturedImages()
{
	std::string unique_dir;
	if ( !MakeUniqueOutDir(unique_dir) ) return false;

	SetOutDirForCapturedImages(unique_dir);

	return true;
}



void
AnimationViewer::MakeAndSetBaseOutDirectoryPath(std::string dir)
{
	base_out_dir_ = dir;
	while ( !base_out_dir_.empty() && base_out_dir_.back() == '/' ) 
		base_out_dir_.erase(base_out_dir_.end()-1);

	while ( !base_out_dir_.empty() && base_out_dir_.front() == '/' ) 
		base_out_dir_.erase(base_out_dir_.begin());

	if ( base_out_dir_.size() == 0 ) return;
	if ( base_out_dir_ == "." ) return;

	// mkdir.
	size_t partition_pos = 0;

	do
	{
		partition_pos = base_out_dir_.find('/', partition_pos+1);

		if ( partition_pos == std::string::npos )
		{
			partition_pos = (int)base_out_dir_.size();
		}

		std::string tmp_d;
		tmp_d.assign(base_out_dir_.begin(), base_out_dir_.begin()+partition_pos);
		std::cout << tmp_d << std::endl;
		#ifdef WIN32
		int r = _mkdir(tmp_d.c_str());
		#else
		int r = mkdir(tmp_d.c_str(), S_IRWXU|S_IRWXG|S_IROTH|S_IXOTH);
		#endif
	} 
	while ( partition_pos != std::string::npos && partition_pos < (int)base_out_dir_.size() );

}

bool
AnimationViewer::MakeUniqueOutDir(std::string &out_dir)
{
	int r;
	unsigned long dir_id = (unsigned long)time(0);

	do 
	{
		std::stringstream tmp_path_s;
		tmp_path_s << base_out_dir_ << "/" << dir_id;
		#ifdef WIN32
		r = _mkdir(tmp_path_s.str().c_str());
				
		if ( r == -1 )
		{
			int e;
			_get_errno(&e);
			if ( e == EEXIST ) 
			{
				printf("base directory is not exist\n");
				return false;
			}
			else
			{
				dir_id++;
			}
		}
		#else
		r = mkdir(tmp_path_s.str().c_str(), S_IRWXU|S_IRWXG|S_IROTH|S_IXOTH);
		if ( r == -1 )
		{
			int e=errno;
			if ( e == EEXIST ) 
			{
				printf("base directory is not exist\n");
				return false;
			}
			else
			{
				dir_id++;
			}
		}

		#endif


	}
	while ( r != 0 );

	std::stringstream out_dir_s;
	out_dir_s << dir_id;
	out_dir = out_dir_s.str();

	return true;
}



};
