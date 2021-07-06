#pragma once

#include "GL/glew.h"
#include "BaseLib/GL4U/GL_Texture.h"

namespace mg
{

class GL_FrameBuffer
{
public:
	GL_FrameBuffer();
	~GL_FrameBuffer();

	/*
	@param target can be one of GL_FRAMEBUFFER, GL_READ_FRAMEBUFFER, or GL_DRAW_FRAMEBUFFER
	*/
	void GenAndBind2D(int w, int h, GLenum target=GL_FRAMEBUFFER);
	void Bind();
	void Unbind();
	void Delete();
	int width() const { return width_; }
	int height() const { return height_; }

	GL_Texture* color_texture() { return texture_; }

protected:
	int width_, height_;
	GL_Texture *texture_;
	GLuint render_buffer_id_;	// For the depth (z-value)
	GLuint frame_buffer_id_;
	GLenum target_;
};


};