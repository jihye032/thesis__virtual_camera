

#pragma once

#include "BaseLib/FLTKU/FL_Glew_Window.h"
//#include "BaseLib/FLTKU/GL3DViewer.h"
#include <map>
#include "BaseLib/FLTKU/Animation.h"
#include "BaseLib/FLTKU/AnimationBrowserListener.h"
#ifdef WIN32
#include <windows.h>
#endif
#include "FL/x.h"

namespace mg
{

class AnimationViewer : public Fl_Glew_Window, public AnimationBrowserListener
{
public:
	AnimationViewer(int x, int y, int w, int h, const char* s=0);
	AnimationViewer(int w, int h, const char* s=0);

	virtual ~AnimationViewer();



	void SetAnimations(const AnimationList &ani_list);
	void AddAnimation(Animation* ani);
	void RemoveAllAnimations();
	AnimationList& animations() { return animations_; }

	virtual void ContentsUpdated() { redraw(); }

	// from AnimationBrowserListener
	virtual void BrowserSelectionsChanged(const AnimationList& ani_list) { SetAnimations(ani_list); }
	virtual void BrowserItemsChanged(const AnimationList& ani_list) {}

	// events
	virtual int handle(int eve);

	// key input mapping
	void SetCallBackKey(void (*cb)(AnimationViewer*, void*), char key, void *data=0);

	// image capture
	#ifdef WIN32
	BITMAPINFO GetBitmapInfo();
	#endif
	int Capture( int ww, int hh, unsigned char* pPixelData );
	void WriteBMP( const char* fn );
	void EnableFrameCapture(bool f) { flag_auto_frame_capture_ = f; }
	void SetOutDirForCapturedImages(std::string dir) { capture_out_dir_ = dir; }
	void SetCaptureImageFilePrefix(std::string f) { capture_image_file_prefix_=f; }
	bool MakeAndSetUniqueDirForCapturedImages();
	void EnableAutoFrameCapture(bool f) { flag_auto_frame_capture_ = f; }

	virtual void MakeAndSetBaseOutDirectoryPath(std::string dir);
	virtual bool MakeUniqueOutDir(std::string &out_dir);

	void flag_ghost_frames(bool f) { flag_ghost_frames_ = f; }
	bool flag_ghost_frames() const { return flag_ghost_frames_; }
	void flag_ground(bool f) { flag_ground_ = f; }
	bool flag_ground() const { return flag_ground_; }
	void flag_line_character(bool f) { flag_line_character_ = f; }
	bool flag_line_character() const { return flag_line_character_; }

	void flag_align_in_grid(bool f) { flag_align_in_grid_ = f; }
	bool flag_align_in_grid() const { return flag_align_in_grid_; }
	void grid_gap(double g) { grid_gap_ = g; }
	double grid_gap() const { return grid_gap_; }
	void grid_max_col(int n) { grid_max_col_ = n; }
	int grid_max_col() const { return grid_max_col_; }

	// Pick (not the best implementation now. I am event not sure if this method should be in this class.
	unsigned int Pick(int m_x, int m_y);
	unsigned int Pick(int m_x, int m_y, cml::vector3d &out_point_on_surface);
	

protected:
	virtual void InitGlew();

	// Draw()
	virtual void Draw();
	virtual void DrawForPicking(int m_x, int m_y);
	

	virtual void DrawGround();
	virtual void DrawContents();
	virtual void DrawContentsForPicking();

	virtual void AutoFrameCapture(int num);
	//virtual void notifyFrameChange();


protected:
	// display
	bool flag_ghost_frames_;
	bool flag_ground_;
	bool flag_line_character_;

	// alignment of charaters
	bool flag_align_in_grid_;
	int grid_max_col_;
	double grid_gap_;

	// Animation
	AnimationList animations_;

	// event
	std::map< char, void (*)(AnimationViewer*, void*) > cb_key_funcs_;
	std::map< char, void* > cb_key_funcs_params_;

	// image capture
	bool flag_auto_frame_capture_;
	std::string capture_out_dir_;
	std::string capture_image_file_prefix_;

	std::string base_out_dir_;


};

};
