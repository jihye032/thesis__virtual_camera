#include "GL_Texture.h"
#include <stdlib.h>
#include <algorithm>

namespace mg
{
GL_Texture::GL_Texture()
{
	id_ = 0;
	target_ = 0;
	levels_ = 1;
	gl_active_texture_id_ = GL_TEXTURE0;

	gl_texture_wrap_s_ = GL_REPEAT;
	gl_texture_wrap_t_ = GL_REPEAT;
}

GL_Texture::~GL_Texture()
{
	if ( id_ != 0 )
		glDeleteTextures(1, &id_);
	id_ = 0;
}

void
GL_Texture::Gen2DAndBind(int width, int height,GLint internal_format)
{
	GenAndBind(GL_TEXTURE_2D, internal_format, width, height, 0, 0);
}

void
GL_Texture::GenAndBind(GLenum target,GLint internal_format,
				GLsizei width, GLsizei height, GLsizei depth,
				GLint border)
{
	if ( id_ != 0 )
		glDeleteTextures(1, &id_);

	target_ = target;

	glGenTextures(1, &id_);
	Bind();

	int long_side = std::max(width, height);
	long_side = std::max(long_side, depth);

	levels_ = (int)log2(long_side);
	if ( levels_<=0 ) levels_=1;
	levels_=1;
	if ( target == GL_TEXTURE_2D )
	{
		glTexStorage2D(target_, levels_, internal_format, width, height);
	}
	else 
	{
		// ????
	}
}

void
GL_Texture::SetData(void *data, GLenum format, GLenum type)
{
	if ( target_ == GL_TEXTURE_2D )
	{
		glTexSubImage2D(target_, 0, 0, 0,
						GetWidth(0), GetHeight(0),
						format, type,
						data);
		
		if ( levels_ >= 2 )
			glGenerateMipmap(target_);
	}
	else
	{
		// ???
	}
}


void
GL_Texture::Bind() const
{
	glActiveTexture(gl_active_texture_id_);
	glBindTexture(target_, id_);
}


bool
GL_Texture::IsBound(GLuint &out_current_bound) const
{

	if ( target_ == GL_TEXTURE_2D )
	{
		GLint b;
		glGetIntegerv(target_, &b);
		out_current_bound = b;
		return (out_current_bound==this->id_);
	}

	else
	{
		// ?????
		return false;
	}
}

bool
GL_Texture::IsBound() const
{
	GLuint current_bound;
	return IsBound(current_bound);
}


GLint 
GL_Texture::GetInternalFormal(GLint level) const
{
	if ( !IsBound() ) Bind();

	GLint result;
	glGetTexLevelParameteriv(target_, level, GL_TEXTURE_INTERNAL_FORMAT, &result);

	return result;
}

GLsizei 
GL_Texture::GetWidth(GLint level) const
{
	if ( !IsBound() ) Bind();

	GLsizei result;
	glGetTexLevelParameteriv(target_, level, GL_TEXTURE_WIDTH, &result);

	return result;
}

GLsizei 
GL_Texture::GetHeight(GLint level) const
{
	if ( !IsBound() ) Bind();

	GLsizei result;
	glGetTexLevelParameteriv(target_, level, GL_TEXTURE_HEIGHT, &result);
 

	return result;
}

GLsizei 
GL_Texture::GetDepth(GLint level) const
{
	if ( !IsBound() ) Bind();

	GLsizei result;
	glGetTexLevelParameteriv(target_, level, GL_TEXTURE_DEPTH, &result);


	return result;
}


};