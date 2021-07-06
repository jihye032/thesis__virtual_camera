
#include "BaseLib/GL4U/GL_FrameBuffer.h"
#include <iostream>

namespace mg
{


GL_FrameBuffer::GL_FrameBuffer()
{
	texture_ = nullptr;
	frame_buffer_id_ = 0;
	render_buffer_id_ = 0;
}

GL_FrameBuffer::~GL_FrameBuffer()
{
	Delete();
}


void
GL_FrameBuffer::GenAndBind2D(int w, int h, GLenum target)
{
	Delete();

	width_ = w;
	height_ = h;

	target_ = target;
	
	// Generate FBO
	glGenFramebuffers(1, &frame_buffer_id_);
	glBindFramebuffer(target, frame_buffer_id_);


	// Generate Color Buffer (Texture)
	texture_ = new mg::GL_Texture();
	texture_->Gen2DAndBind(w, h);
	texture_->id();

	// Attach the Color Buffer to FBO
	glFramebufferTexture2D(target, GL_COLOR_ATTACHMENT0, texture_->GetTarget(), texture_->id(), 0);


	// Generate Render Buffer (for Depth buffer)
	glGenRenderbuffers(1, &render_buffer_id_);
	glBindRenderbuffer(GL_RENDERBUFFER, render_buffer_id_);
	glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, w, h);

	// Attach the Depth Buffer to FBO
	glFramebufferRenderbuffer(GL_DRAW_BUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, render_buffer_id_);

	if ( glCheckFramebufferStatus(target) != GL_FRAMEBUFFER_COMPLETE )
	{
		std::cerr << "Error: GL_FrameBuffer::GenAndBind2D " << std::endl;
		exit(0);
	}
}


void
GL_FrameBuffer::Bind()
{
	glBindFramebuffer(target_, frame_buffer_id_);
}


void
GL_FrameBuffer::Unbind()
{
	glBindFramebuffer(target_, 0);
}



void
GL_FrameBuffer::Delete()
{
	if ( texture_ != nullptr )
	{
		delete texture_;
		texture_ = nullptr;
	}

	if ( render_buffer_id_ != 0 )
	{
		glDeleteRenderbuffers(1, &render_buffer_id_);
		render_buffer_id_ = 0;
	}

	if ( frame_buffer_id_ != 0 ) 
	{
		glDeleteFramebuffers(1, &frame_buffer_id_);
		frame_buffer_id_ = 0;
	}
}

};