#include "BaseLib/BaseLib.h"
#include "BaseLib/GL4U/GL_Renderer.h"
#include "BaseLib/GL4U/GL_ResourceManager.h"
#include "BaseLib/GL4U/GL_Font.h"
#include <string>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <algorithm>
#include <ft2build.h>
#include FT_FREETYPE_H


/* Ref: https://en.wikibooks.org/wiki/OpenGL_Programming/Modern_OpenGL_Tutorial_Text_Rendering_02
*/


namespace mg
{
static FT_Face g_face;
static int g_font_size;
static struct {
	float ax;	// advance.x
	float ay;	// advance.y

	float bw;	// bitmap.width;
	float bh;	// bitmap.height;

	float bl;	// bitmap_left;
	float bt;	// bitmap_top;

	float tx;	// x offset of glyph in texture coordinates
	float ty;	// y offset of glyph in texture coordinates
} g_char[128];		// character information


GL_Font::GL_Font(int font_size)
{
	std::string defalut_fontfile = GetBaseLibResourcePath();
	defalut_fontfile += "fonts/FreeSans.ttf";
	Init(defalut_fontfile, font_size);
}

GL_Font::GL_Font(std::string font_ttf_file, int font_size)
{
	Init(font_ttf_file, font_size);
}

void
GL_Font::Init(std::string font_ttf_file, int font_size)
{
	//tex_id_ = 0;
	vbo_ = 0 ;
	vao_ = 0;
	texture_ = 0;

	FT_Library ft;

	/* Initialize the FreeType2 library */
	if (FT_Init_FreeType(&ft)) {
		fprintf(stderr, "Could not init freetype library\n");
		std::exit(0);
	}


	/* Load a font */
	if (FT_New_Face(ft, font_ttf_file.c_str(), 0, &g_face)) {
		fprintf(stderr, "Could not open font %s\n", font_ttf_file.c_str());
		std::exit(0);
	}

	g_font_size = font_size;
	CreateTexture();


	// Create VBO
	vbo_ = new GL_VBOGroup;
	vao_ = new GL_VAO(vbo_);


	/* Relase Font Resource */
	FT_Done_Face(g_face);
	FT_Done_FreeType(ft);
}

GL_Font::~GL_Font()
{
	//if ( tex_id_ != 0 ) glDeleteTextures(1, &tex_id_);
	if ( vao_ != 0 ) delete vao_;
	if ( vbo_ != 0 ) delete vbo_;
	if ( texture_ != 0 ) delete texture_;
}

void 
GL_Font::CreateTexture()
{
	const int MAXWIDTH = 1024;
	

	unsigned int w;			// width of texture in pixels
	unsigned int h;			// height of texture in pixels


	FT_Set_Pixel_Sizes(g_face, 0, g_font_size);
	FT_GlyphSlot g = g_face->glyph;

	unsigned int roww = 0;
	unsigned int rowh = 0;
	w = 0;
	h = 0;

	memset(g_char, 0, sizeof g_char);

	/* Find minimum size for a texture holding all visible ASCII characters */
	for (int i = 32; i < 128; i++) {
		if (FT_Load_Char(g_face, i, FT_LOAD_RENDER)) {
			fprintf(stderr, "Loading character %c failed!\n", i);
			continue;
		}
		if (roww + g->bitmap.width + 1 >= MAXWIDTH) {
			w = std::max(w, roww);
			h += rowh;
			roww = 0;
			rowh = 0;
		}
		roww += g->bitmap.width + 1;
		rowh = std::max(rowh, (unsigned int)g->bitmap.rows);
	}

	w = std::max(w, roww);
	h += rowh;

	texture_ = new GL_Texture;
	texture_->Gen2DAndBind(w, h, GL_ALPHA);
	//glActiveTexture(texture_->gl_active_texture_id());
	/*glGenTextures(1, &tex_id_);
	glBindTexture(GL_TEXTURE_2D, tex_id_);
	glUniform1i(uniform_tex, 0);*/

	glTexImage2D(GL_TEXTURE_2D, 0, GL_ALPHA, w, h, 0, GL_ALPHA, GL_UNSIGNED_BYTE, 0);

	/* We require 1 byte alignment when uploading texture data */
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

	/* Clamping to edges is important to prevent artifacts when scaling */
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

	/* Linear filtering usually looks best for text */
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	/* Paste all glyph bitmaps into the texture, remembering the offset */
	int ox = 0;
	int oy = 0;

	rowh = 0;

	for (int i = 32; i < 128; i++) {
		if (FT_Load_Char(g_face, i, FT_LOAD_RENDER)) {
			fprintf(stderr, "Loading character %c failed!\n", i);
			continue;
		}

		if (ox + g->bitmap.width + 1 >= MAXWIDTH) {
			oy += rowh;
			rowh = 0;
			ox = 0;
		}

		glTexSubImage2D(GL_TEXTURE_2D, 0, ox, oy, g->bitmap.width, g->bitmap.rows, GL_ALPHA, GL_UNSIGNED_BYTE, g->bitmap.buffer);
		g_char[i].ax = g->advance.x >> 6;
		g_char[i].ay = g->advance.y >> 6;

		g_char[i].bw = g->bitmap.width;
		g_char[i].bh = g->bitmap.rows;

		g_char[i].bl = g->bitmap_left;
		g_char[i].bt = g->bitmap_top;

		g_char[i].tx = ox / (float)w;
		g_char[i].ty = oy / (float)h;

		rowh = std::max(rowh, g->bitmap.rows);
		ox += g->bitmap.width + 1;
	}


	fprintf(stderr, "Generated a %d x %d (%d kb) texture atlas\n", w, h, w * h / 1024);
}

void
GL_Font::DrawString(std::string str)
{
	double x = 0;
	double y = 0;
	double sx = 1.0;
	double sy = 1.0;

	const uint8_t *p;
	const char *text = str.c_str();

	/* Use the texture containing the atlas */
	//glBindTexture(GL_TEXTURE_2D, tex_id_);
	vao_->Bind();
	vbo_->BindVBO();
	//glUniform1i(uniform_tex, 0);

	/* Set up the VBO for our vertex data */
	/*glEnableVertexAttribArray(attribute_coord);
	glBindBuffer(GL_ARRAY_BUFFER, vbo_id_);
	glVertexAttribPointer(attribute_coord, 4, GL_FLOAT, GL_FALSE, 0, 0);*/
	vbo_->drawing_mode(GL_TRIANGLES);
	vbo_->BeginVertex();

	//point *coords = new point[6 * strlen(text)];
	int c = 0;

	int texture_h = texture_->GetHeight();
	int texture_w = texture_->GetWidth();

	/* Loop through all characters */
	for (p = (const uint8_t *)text; *p; p++) {
		/* Calculate the vertex and texture coordinates */
		float x2 = (float)(x + g_char[*p].bl * sx);
		float y2 = (float)(-y - g_char[*p].bt * sy);
		float w = (float)(g_char[*p].bw * sx);
		float h = (float)(g_char[*p].bh * sy);

		/* Advance the cursor to the start of the next character */
		x += g_char[*p].ax * sx;
		y += g_char[*p].ay * sy;

		/* Skip glyphs that have no pixels */
		if (!w || !h)
			continue;

		vbo_->glUV({g_char[*p].tx                    , g_char[*p].ty                    });
		vbo_->glVertex({x2    , -y2    , 0.f});

		vbo_->glUV({g_char[*p].tx                    , g_char[*p].ty + g_char[*p].bh / texture_h});
		vbo_->glVertex({x2    , -y2 - h, 0.f});

		vbo_->glUV({g_char[*p].tx + g_char[*p].bw / texture_w, g_char[*p].ty                    });
		vbo_->glVertex({x2 + w, -y2    , 0.f});

		



		vbo_->glUV({g_char[*p].tx + g_char[*p].bw / texture_w, g_char[*p].ty                    });
		vbo_->glVertex({x2 + w, -y2    , 0.f});

		vbo_->glUV({g_char[*p].tx                    , g_char[*p].ty + g_char[*p].bh / texture_h});
		vbo_->glVertex({x2    , -y2 - h, 0.f});

		vbo_->glUV({g_char[*p].tx + g_char[*p].bw / texture_w, g_char[*p].ty + g_char[*p].bh / texture_h});
		vbo_->glVertex({x2 + w, -y2 - h, 0.f});

		c+=6;
		/*coords[c++] = {			x2    , -y2    , g_char[*p].tx                    , g_char[*p].ty                     };
		coords[c++] = {			x2 + w, -y2    , g_char[*p].tx + g_char[*p].bw / w, g_char[*p].ty                     };
		coords[c++] = {			x2    , -y2 - h, g_char[*p].tx                    , g_char[*p].ty + g_char[*p].bh / h};
		coords[c++] = {			x2 + w, -y2    , g_char[*p].tx + g_char[*p].bw / w, g_char[*p].ty                    };
		coords[c++] = {			x2    , -y2 - h, g_char[*p].tx                    , g_char[*p].ty + g_char[*p].bh / h};
		coords[c++] = {			x2 + w, -y2 - h, g_char[*p].tx + g_char[*p].bw / w, g_char[*p].ty + g_char[*p].bh / h};*/
	}

	/*vbo_->glUV({0, 1});
	vbo_->glVertex({0, 0, 0});

	vbo_->glUV({1, 1});
	vbo_->glVertex({100, 0, 0});

	vbo_->glUV({1, 0});
	vbo_->glVertex({100, 100, 0});


	vbo_->glUV({0, 1});
	vbo_->glVertex({0, 0, 0});

	vbo_->glUV({1, 0});
	vbo_->glVertex({100, 100, 0});

	vbo_->glUV({0, 0});
	vbo_->glVertex({0, 100, 0});*/

	vbo_->EndVertex();

	vao_->UpdateVboGroupAttris();
	
	/* Draw all the character on the screen in one go */
	// glBufferData(GL_ARRAY_BUFFER, sizeof coords, coords, GL_DYNAMIC_DRAW);
	//glDrawArrays(GL_TRIANGLES, 0, c);

	texture_->Bind();
	vao_->Draw();

	//glDisableVertexAttribArray(attribute_coord);
	//delete[] coords;
}


};

