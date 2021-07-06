#pragma once

#include <string>
#include "GL/glew.h"
#include "BaseLib/GL4U/GL_VBOVAO.h"

/*
GL_Font class represents resources and methods to render a text in GL space.
A true type font is loaded by using FreeType lib. 
The ascci character set is rendered in a GL texture. 
When a string is requreded to render, the string is composed by disposing characters of the texture on a rectectle.
*/
namespace mg
{

class GL_Font
{
friend class GL_ResourceManager;

protected:
	GL_Font(std::string font_ttf_file, int font_size=12);
	GL_Font(int font_size=12);
	void CreateTexture();

public:
	void Init(std::string font_ttf_file, int font_size);
	virtual ~GL_Font();
	void DrawString(std::string str);

protected:
	GL_VBOGroup *vbo_;
	GL_VAO *vao_;
	GL_Texture *texture_;
};

};