

#include "BaseLib/BaseLib.h"
#include "BaseLib/GL4U/GL_ShaderProgram.h"
#include "BaseLib/GL4U/GL_Material.h"
#include "BaseLib/GL4U/GL_Texture.h"
#include <sstream>
#include <fstream>
#include <iostream>

namespace mg
{

GL_ShaderProgram::GL_ShaderProgram()
{
	program_handle_ = 0;
	never_used_yet_ = true;
}

GL_ShaderProgram::~GL_ShaderProgram()
{
	Delete();
}

bool
GL_ShaderProgram::ReadSourceFromFile(const std::string &in_filename, std::string &out_src)
{
	out_src.clear();
	std::ifstream fin(in_filename.c_str());

	if ( fin.fail() )
	{
		std::cerr << "Unable to open file '" << in_filename << "'" << std::endl;
		return false;
	}
	else
	{
		while (true)
		{
			char c = char(fin.get());
			if (!fin)
				return fin.eof();
			out_src += c;
		}
	}
}


GLuint 
GL_ShaderProgram::CreateFromFiles(const std::string &v_shader_file, const std::string &f_shader_file)
{
	std::string v_shader_src;
	std::string f_shader_src;

	
	if ( !ReadSourceFromFile(v_shader_file, v_shader_src) )
	{
		return 0;
	}

	if ( !ReadSourceFromFile(f_shader_file, f_shader_src) )
	{
		return 0;
	}

	program_handle_ = CreateFromSource(v_shader_src, f_shader_src);

	return program_handle_;
}

GLuint
GL_ShaderProgram::CreateFromSource(const std::string &v_shader_src, const std::string &f_shader_src)
{
	GLuint program = glCreateProgram();

	// vertex shader
	GLuint v_shader = glCreateShader(GL_VERTEX_SHADER);
	{
		const GLchar *v_src = v_shader_src.c_str();
		glShaderSource(v_shader, 1, &v_src, NULL);
		glCompileShader(v_shader);

		GLint compiled;
		glGetShaderiv(v_shader, GL_COMPILE_STATUS, &compiled);

		if (!compiled) 
		{
			GLsizei len;
			glGetShaderiv( v_shader, GL_INFO_LOG_LENGTH, &len );

			GLchar* log = new GLchar[len+1];
			glGetShaderInfoLog( v_shader, len, &len, log );
			std::cerr << "Shader compilation failed: " << log << std::endl;
			delete [] log;
			return 0;
		}
		glAttachShader(program, v_shader);
	}

	// fragment shader
	GLuint f_shader = glCreateShader(GL_FRAGMENT_SHADER);
	{
		const GLchar *f_src = f_shader_src.c_str();
		glShaderSource(f_shader, 1, &f_src, NULL);
		glCompileShader(f_shader);

		GLint compiled;
		glGetShaderiv(f_shader, GL_COMPILE_STATUS, &compiled);

		if (!compiled) 
		{
			GLsizei len;
			glGetShaderiv( f_shader, GL_INFO_LOG_LENGTH, &len );

			GLchar* log = new GLchar[len+1];
			glGetShaderInfoLog( f_shader, len, &len, log );
			std::cerr << "Shader compilation failed: " << log << std::endl;
			delete [] log;
			return 0;
		}
		glAttachShader(program, f_shader);
	}


	glLinkProgram(program);

	GLint linked;
	glGetProgramiv(program, GL_LINK_STATUS, &linked);
	if (!linked) 
	{
		GLsizei len;
		glGetProgramiv( program, GL_INFO_LOG_LENGTH, &len );

		GLchar* log = new GLchar[len+1];
		glGetProgramInfoLog( program, len, &len, log );
		std::cerr << "Shader linking failed: " << log << std::endl;
		delete [] log;

		glDeleteShader(v_shader);
		glDeleteShader(f_shader);
		glDeleteProgram(program);

		return 0;
	}

	return program;
}

void
GL_ShaderProgram::CreateDefaultProgram()
{
	std::string v_sharder_file;
	v_sharder_file = GetBaseLibResourcePath();
	v_sharder_file += "shaders/defalut_vshader.glsl";

	std::string f_sharder_file;
	f_sharder_file = GetBaseLibResourcePath();
	f_sharder_file += "shaders/defalut_fshader.glsl";

	program_handle_ = CreateFromFiles(v_sharder_file, f_sharder_file);
	
}

////////////////////////////////////////////////////////////
// Set Default Vertex Attributes

void 
GL_ShaderProgram::SetDefaultVertexColor(double r, double g, double b, double a)
{
	if ( !IsUsing() ) UseProgram();
	glVertexAttrib4d(V_COLOR_LOC, r, g, b, a);
}

void 
GL_ShaderProgram::SetDefaultVertexColor(const cml::vector4f &c)
{
	SetDefaultVertexColor(c[0], c[1], c[2], c[3]);
}






////////////////////////////////////////////////////////////
// Set Unifrom Values

void
GL_ShaderProgram::SetShadingMode(int m)
{
	if (!IsUsing()) UseProgram();
	GLint shading_mode_loc = glGetUniformLocation(program_handle_, "shading_mode");
	if (shading_mode_loc < 0) return;

	glUniform1i(shading_mode_loc, m);
}


////////////////////////////////////////////////////////////
// Set Unifrom Values : Matrices

void
GL_ShaderProgram::SetProjectionMatrix(cml::matrix44f_c &m44_c)
{
	SetProjectionMatrix(m44_c.data());
}

void
GL_ShaderProgram::SetViewMatrix(cml::matrix44f_c &m44_c)
{
	SetViewMatrix(m44_c.data());
}

void
GL_ShaderProgram::SetModelMatrix(cml::matrix44f_c &m44_c)
{
	SetModelMatrix(m44_c.data());
}


void
GL_ShaderProgram::SetProjectionMatrix(const float *m44)
{
	if (!IsUsing()) UseProgram();
	glUniformMatrix4fv(projection_matrix_loc_, 1, GL_FALSE, m44);
}

void
GL_ShaderProgram::SetViewMatrix(const float *m44)
{
	if (!IsUsing()) UseProgram();
	glUniformMatrix4fv(view_matrix_loc_, 1, GL_FALSE, m44);
}

void
GL_ShaderProgram::SetModelMatrix(const float *m44)
{
	if (!IsUsing()) UseProgram();
	glUniformMatrix4fv(model_matrix_loc_, 1, GL_FALSE, m44);
}


////////////////////////////////////////////////////////////
// Set Unifrom Values : Materials

void
GL_ShaderProgram::SetMaterial(const GL_Material &m, bool flag_texture)
{
	if (!IsUsing()) UseProgram();

	SetPhongMaterial(m.ambient_color_f(), m.diffuse_color_f(), m.specular_color_f(), m.shininess_f());
	SetDefaultVertexColor(m.diffuse_color_f());

	EnableVertexColor(m.flag_use_vertex_color());
	//EnableTexture(m.flag_texture());

	if ( !m.flag_shading() )
	{
		SetShadingMode(NO_SHADING);
	}
	else if ( m.flag_flat_shading() )
	{
		SetShadingMode(FLAT_SHADING);
	}
	else
	{
		SetShadingMode(PHONG_SHADING);
	}

	if ( flag_texture && m.flag_texture() && m.CountTextures()>0  )
	{
		EnableTexture(true);

		for ( int i=0; i<m.CountTextures(); i++ )
		{
			SetTexture( *(m.texture(i)) );
		}
	}
	else
	{
		EnableTexture(false);

	}


}

void
GL_ShaderProgram::SetPhongMaterial(const cml::vector4f &k_a, const cml::vector4f &k_d, const cml::vector4f &k_s, const float shinness)
{
	if (!IsUsing()) UseProgram();

	glUniform4fv(glGetUniformLocation(program_handle_, "K_a"), 1, k_a.data());
	glUniform4fv(glGetUniformLocation(program_handle_, "K_d"), 1, k_d.data());
	glUniform4fv(glGetUniformLocation(program_handle_, "K_s"), 1, k_s.data());
	glUniform1f(glGetUniformLocation(program_handle_, "shininess_n"), shinness);
}

void
GL_ShaderProgram::SetTexture(const GL_Texture &t)
{
	t.Bind();
}

void
GL_ShaderProgram::EnableVertexColor(bool f)
{
	if (!IsUsing()) UseProgram();

	glUniform1i(glGetUniformLocation(program_handle_, "flag_use_vertex_color"), (int)f);
}

void
GL_ShaderProgram::EnableTexture(bool f)
{
	if (!IsUsing()) UseProgram();
	glUniform1i(glGetUniformLocation(program_handle_, "flag_texture"), (int)f);
}



////////////////////////////////////////////////////////////
// Set Unifrom Values : Lighting

void 
GL_ShaderProgram::SetAmbientLightIntensity(float l)
{
	if (!IsUsing()) UseProgram();

	glUniform1f(glGetUniformLocation(program_handle_, "I_a"), l);
}

void
GL_ShaderProgram::EnableLight(int light_i, bool f)
{
	if (!IsUsing()) UseProgram();

	std::string fs_val_name = "flag_lights[" + std::to_string(light_i) + "]";

	glUniform1i(glGetUniformLocation(program_handle_, ( "flag_lights[" + std::to_string(light_i) + "]").c_str()), (int)f);
}

void
GL_ShaderProgram::SetLight(int light_id, const GL_Light &l)
{
	SetLight(light_id, l.type, l.dir, l.position, l.intensity, l.cos_cutoff);
}

void
GL_ShaderProgram::SetLight(int light_id, int type, cml::vector3f dir, cml::vector3f position, cml::vector4f intensity, float cos_cutoff)
{
	if (!IsUsing()) UseProgram();

	glUniform1i(glGetUniformLocation(program_handle_, ("lights[" + std::to_string(light_id) + "].type").c_str()), type);
	glUniform3fv(glGetUniformLocation(program_handle_, ("lights[" + std::to_string(light_id) + "].dir").c_str()), 1, dir.data());
	glUniform3fv(glGetUniformLocation(program_handle_, ("lights[" + std::to_string(light_id) + "].position").c_str()), 1, position.data());
	glUniform4fv(glGetUniformLocation(program_handle_, ("lights[" + std::to_string(light_id) + "].intensity").c_str()), 1, intensity.data());
	glUniform1f(glGetUniformLocation(program_handle_, ("lights[" + std::to_string(light_id) + "].cos_cutoff").c_str()), cos_cutoff);
}

/*
void 
GL_ShaderProgram::SetLightPosition(int light_id, cml::vector3f position)
{
	if (!IsUsing()) UseProgram();
	glUniform3fv(glGetUniformLocation(program_handle_, ("lights[" + std::to_string(light_id) + "].position").c_str()), 1, position.data());
}

void 
GL_ShaderProgram::SetLightDirection(int light_id, cml::vector3f dir)
{
	if (!IsUsing()) UseProgram();
	glUniform3fv(glGetUniformLocation(program_handle_, ("lights[" + std::to_string(light_id) + "].dir").c_str()), 1, dir.data());
}

void 
GL_ShaderProgram::SetLightPosDir(int light_id, cml::vector3f position, cml::vector3f dir)
{
	if (!IsUsing()) UseProgram();
	glUniform3fv(glGetUniformLocation(program_handle_, ("lights[" + std::to_string(light_id) + "].position").c_str()), 1, position.data());
	glUniform3fv(glGetUniformLocation(program_handle_, ("lights[" + std::to_string(light_id) + "].dir").c_str()), 1, dir.data());
}
*/

GL_Light
GL_ShaderProgram::GetLight(int light_id) const
{
	GL_Light l;
	float tmp[4];

	glGetUniformiv(program_handle_, glGetUniformLocation(program_handle_, ("lights[" + std::to_string(light_id) + "].type").c_str()), &l.type);
	glGetUniformfv(program_handle_, glGetUniformLocation(program_handle_, ("lights[" + std::to_string(light_id) + "].dir").c_str()), tmp);
	l.dir.set(tmp[0], tmp[1], tmp[2]);

	glGetUniformfv(program_handle_, glGetUniformLocation(program_handle_, ("lights[" + std::to_string(light_id) + "].position").c_str()), tmp);
	l.position.set(tmp[0], tmp[1], tmp[2]);

	glGetUniformfv(program_handle_, glGetUniformLocation(program_handle_, ("lights[" + std::to_string(light_id) + "].intensity").c_str()), tmp);
	l.intensity.set(tmp[0], tmp[1], tmp[2], tmp[3]);

	glGetUniformfv(program_handle_, glGetUniformLocation(program_handle_, ("lights[" + std::to_string(light_id) + "].cos_cutoff").c_str()), &l.cos_cutoff);

	return l;
}

void
GL_ShaderProgram::DisableAllLight()
{
	for (int i = 0; i<max_num_lights_; i++)
	{
		EnableLight(i, false);
	}
}




////////////////////////////////////////////////////////////
// Set Unifrom Values : Skinning


void
GL_ShaderProgram::EnableSkinning(bool f)
{
	if ( !IsUsing() ) UseProgram();

	glUniform1i( glGetUniformLocation(program_handle_, "flag_skinning"), (int)f );
}





bool
GL_ShaderProgram::IsSkinningOn()
{
	if (!IsUsing()) UseProgram();

	int result;
	glGetUniformiv(program_handle_, glGetUniformLocation(program_handle_, "flag_skinning"), &result);

	return (bool)result;
}

void
GL_ShaderProgram::UseProgram()
{
	if (program_handle_ != 0)
	{
		glUseProgram(program_handle_);

		if ( never_used_yet_ )
		{

			// Get Unifrom Locations
			projection_matrix_loc_ = glGetUniformLocation(program_handle_, "projection_matrix");
			view_matrix_loc_ = glGetUniformLocation(program_handle_, "view_matrix");
			model_matrix_loc_ = glGetUniformLocation(program_handle_, "model_matrix");
			bone_matrix_loc_ = glGetUniformLocation(program_handle_, "bone_matrices");
			shadowmap_tex_bias_light_pov_matrix_loc_ = glGetUniformLocation(program_handle_, "shadowmap_tex_bias_light_pov_matrix");
			tex7_shadowmap_loc_ = glGetUniformLocation(program_handle_, "tex7_shadowmap");


			// Set Default Values : Vertex Atributes
			glVertexAttrib4d(V_POSITION_LOC, 0.0, 0.0, 0.0, 1.0);
			glVertexAttrib3d(V_NORMAL_LOC, 0.0, 0.0, 0.0);
			glVertexAttrib2d(V_TEX_COORD_LOC, 0.0, 0.0);
			glVertexAttrib4d(V_COLOR_LOC, 1.0, 1.0, 1.0, 1.0);
			glVertexAttribI4ui(V_BONE_ID_LOC, 0, 0, 0, 0);
			glVertexAttrib4d(V_BONE_WEIGHT_LOC, 0.0, 0.0, 0.0, 0.0);

			glUniform1i(glGetUniformLocation(program_handle_, "tex0"), 0);


			// Set Default Values : Skinning
			{
				EnableSkinning(false);

				float* bone_matrices = new float[100 * 4 * 4];
				for (int i = 0; i < 100; i++)
				{
					for (int j = 0; j < 4; j++)
					{
						for (int k = 0; k < 4; k++)
						{
							if (j == k)
								bone_matrices[i * 4 * 4 + j * 4 + k] = 1.0f;
							else
								bone_matrices[i * 4 * 4 + j * 4 + k] = 0.0f;
						}
					}
				}

				glUniformMatrix4fv(bone_matrix_loc_, 100, GL_FALSE, bone_matrices);
				delete[] bone_matrices;
			}

			// Set Default Values : Lighting
			{
				max_num_lights_ = glGetUniformLocation(program_handle_, "num_lights");;
				DisableAllLight();

				/*for ( int i=0; i<max_num_lights_; i++ )
				{
					SetLight(i, 0, cml::vector3f(0.f, 0.f, -1.f));
				}
				EnableLight(0, true);*/
			}

			never_used_yet_ = false;
		}
	}
}

GLint
GL_ShaderProgram::GetViewMatrixLoc()
{
	return glGetUniformLocation(program_handle_, "view_matrix");
}

GLint
GL_ShaderProgram::GetModelMatrixLoc()
{
	return glGetUniformLocation(program_handle_, "model_matrix");
}

GLint
GL_ShaderProgram::GetProjectionMatrixLoc()
{
	return glGetUniformLocation(program_handle_, "projection_matrix");
}








////////////////////////////////////////////////////////
// Bone

GLint 
GL_ShaderProgram::GetBoneMatrixLoc(int bone_id) 
{
	if ( !IsUsing() ) UseProgram();
	return bone_matrix_loc_+bone_id;

	std::stringstream sstr;
	sstr << "bone_matrices[" << bone_id << "]";
	std::string name;
	name = sstr.str();
	return glGetUniformLocation(program_handle_, name.c_str());
}


void 
GL_ShaderProgram::SetBoneMatrix(int bone_id, cml::matrix44f_c &m44_c)
{
	SetBoneMatrix(bone_id, m44_c.data());
}

void 
GL_ShaderProgram::SetBoneMatrix(int bone_id, float *m44)
{
	if ( !IsUsing() ) UseProgram();
	glUniformMatrix4fv(GetBoneMatrixLoc(bone_id), 1, GL_FALSE, m44);
}



////////////////////////////////////////////////////////
// Shadow

void 
GL_ShaderProgram::SetShadowmapLightProjectionViewMatrix(cml::matrix44f_c &m44_c)
{
	SetShadowmapLightProjectionViewMatrix(m44_c.data());
}

void 
GL_ShaderProgram::SetShadowmapLightProjectionViewMatrix(const float *m44)
{
	if ( !IsUsing() ) UseProgram();
	glUniformMatrix4fv( glGetUniformLocation(program_handle_, "shadowmap_tex_bias_light_pov_matrix"), 1, GL_FALSE, m44);
}

void 
GL_ShaderProgram::EnableShadowMap(bool f)
{
	if ( !IsUsing() ) UseProgram();
	glUniform1i( glGetUniformLocation(program_handle_, "flag_shadowmap"), (int)f );
}

void
GL_ShaderProgram::BindShadowmapTextureId(GLuint texture_id)
{
	glUniform1i(tex7_shadowmap_loc_, 7);
	glActiveTexture(GL_TEXTURE7);
	glBindTexture(GL_TEXTURE_2D, texture_id);
}




////////////////////////////////////////////////////////
bool
GL_ShaderProgram::IsUsing() const
{
	GLint cur_program_handle_;
	glGetIntegerv(GL_CURRENT_PROGRAM, &cur_program_handle_);

	if ( cur_program_handle_ == (GLint)program_handle_ ) return true;

	return false;
}


void
GL_ShaderProgram::Delete()
{
	if ( program_handle_ != 0 )
	{
		if ( IsUsing() )
			glUseProgram(0);

		glDeleteShader(program_handle_);
		never_used_yet_ = true;
	}
}




////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//// Genernal Get-Set methods

//// Set single values.
void 
GL_ShaderProgram::SetUniformMat4(std::string var_name, cml::matrix44f_c &m)
{
	if (!IsUsing()) UseProgram();

	glUniformMatrix4fv( 
		glGetUniformLocation(program_handle_, var_name.c_str()), 
		1, 
		GL_FALSE, 
		m.data()
	);

};

template<class T>
void 
GL_ShaderProgram::SetUniform(std::string var_name, T d)
{
	if (!IsUsing()) UseProgram();

	float f = (float)d;

	glUniform1f( glGetUniformLocation(program_handle_, var_name.c_str()), d );
};



template<>
void 
GL_ShaderProgram::SetUniform(std::string var_name, cml::matrix44f_c d)
{
	if (!IsUsing()) UseProgram();
	
	glUniformMatrix4fv( glGetUniformLocation(program_handle_, var_name.c_str()), 1, GL_FALSE, d.data() );

};


template<>
void 
GL_ShaderProgram::SetUniform(std::string var_name, bool d)
{
	if (!IsUsing()) UseProgram();

	glUniform1i( glGetUniformLocation(program_handle_, var_name.c_str()), d );

};

template<>
void 
GL_ShaderProgram::SetUniform(std::string var_name, int d)
{
	if (!IsUsing()) UseProgram();

	glUniform1i( glGetUniformLocation(program_handle_, var_name.c_str()), d );

};

template<>
void 
GL_ShaderProgram::SetUniform(std::string var_name, unsigned int d)
{
	if (!IsUsing()) UseProgram();

	glUniform1ui( glGetUniformLocation(program_handle_, var_name.c_str()), d );

};

template<>
void 
GL_ShaderProgram::SetUniform(std::string var_name, float d)
{
	if (!IsUsing()) UseProgram();

	glUniform1f( glGetUniformLocation(program_handle_, var_name.c_str()), d );
};

template<>
void 
GL_ShaderProgram::SetUniform(std::string var_name, double d)
{
	if (!IsUsing()) UseProgram();

	glUniform1d( glGetUniformLocation(program_handle_, var_name.c_str()), d );
};

template<>
void 
GL_ShaderProgram::SetUniform(std::string var_name, cml::vector3f d)
{
	if (!IsUsing()) UseProgram();
	glUniform3fv( glGetUniformLocation(program_handle_, var_name.c_str()), 1, d.data() );
};

template<>
void 
GL_ShaderProgram::SetUniform(std::string var_name, cml::vector3d d)
{
	if (!IsUsing()) UseProgram();

	glUniform3dv( glGetUniformLocation(program_handle_, var_name.c_str()), 1, d.data() );
};

template<>
void 
GL_ShaderProgram::SetUniform(std::string var_name, cml::vector4f d)
{
	if (!IsUsing()) UseProgram();

	glUniform4fv( glGetUniformLocation(program_handle_, var_name.c_str()), 1, d.data() );
};

template<>
void 
GL_ShaderProgram::SetUniform(std::string var_name, cml::vector4d d)
{
	if (!IsUsing()) UseProgram();

	glUniform4dv( glGetUniformLocation(program_handle_, var_name.c_str()), 1, d.data() );
};

//// Set point of arrays
template<class T>
void 
GL_ShaderProgram::SetUniformV(std::string var_name, int size, T *d)
{
	if (!IsUsing()) UseProgram();

	float* f = (float*)d;

	glUniform1fv( glGetUniformLocation(program_handle_, var_name.c_str()), size, f	);

};


template<>
void 
GL_ShaderProgram::SetUniformV(std::string var_name, int size, cml::matrix44f_c *d)
{
	if (!IsUsing()) UseProgram();

	glUniformMatrix4fv( 
		glGetUniformLocation(program_handle_, var_name.c_str()), 
		size, 
		GL_FALSE, 
		d[0].data()
	);

};

template<>
void 
GL_ShaderProgram::SetUniformV(std::string var_name, int size, int *d)
{
	if (!IsUsing()) UseProgram();

	glUniform1iv( glGetUniformLocation(program_handle_, var_name.c_str()), size, d );
};

template<>
void 
GL_ShaderProgram::SetUniformV(std::string var_name, int size, unsigned int *d)
{
	if (!IsUsing()) UseProgram();

	glUniform1uiv( glGetUniformLocation(program_handle_, var_name.c_str()), size, d );

};

template<>
void 
GL_ShaderProgram::SetUniformV(std::string var_name, int size, float *d)
{
	if (!IsUsing()) UseProgram();

	glUniform1fv( glGetUniformLocation(program_handle_, var_name.c_str()), size, d );
};

template<>
void 
GL_ShaderProgram::SetUniformV(std::string var_name, int size, double *d)
{
	if (!IsUsing()) UseProgram();

	glUniform1dv( glGetUniformLocation(program_handle_, var_name.c_str()), size, d );
};


template<>
void 
GL_ShaderProgram::SetUniformV(std::string var_name, int size, cml::vector3f *d)
{
	if (!IsUsing()) UseProgram();

	glUniform3fv( glGetUniformLocation(program_handle_, var_name.c_str()), size, d[0].data() );
};

template<>
void 
GL_ShaderProgram::SetUniformV(std::string var_name, int size, cml::vector3d *d)
{
	if (!IsUsing()) UseProgram();

	glUniform3dv( glGetUniformLocation(program_handle_, var_name.c_str()), size, d[0].data() );
};

template<>
void 
GL_ShaderProgram::SetUniformV(std::string var_name, int size, cml::vector4f *d)
{
	if (!IsUsing()) UseProgram();

	glUniform4fv( glGetUniformLocation(program_handle_, var_name.c_str()), size, d[0].data() );
};

template<>
void 
GL_ShaderProgram::SetUniformV(std::string var_name, int size, cml::vector4d *d)
{
	if (!IsUsing()) UseProgram();

	glUniform4dv( glGetUniformLocation(program_handle_, var_name.c_str()), size, d[0].data() );
};





};



