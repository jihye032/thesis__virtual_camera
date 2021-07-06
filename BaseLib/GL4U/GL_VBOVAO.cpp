


#include "BaseLib/GL4U/GL_VBOVAO.h"
#include "BaseLib/GL4U/GL_RenderableObj.h"
#include "BaseLib/GL4U/GL_ShaderProgram.h"
#include "BaseLib/GL4U/GL_Material.h"


namespace mg
{

GL_VBO::GL_VBO()
{
	vbo_id_ = 0;
}

GL_VBO::GL_VBO(GLenum data_type, int vec_dim, int num_of_vectors, void *data, GLenum target, GLenum usage)
{
	vbo_id_ = 0;

	GenAndBind(data_type, vec_dim, num_of_vectors, data, target, usage);
}

GL_VBO::~GL_VBO()
{
	Delete();
}

void 
GL_VBO::GenAndBind(GLenum data_type, int vec_dim, int num_of_vectors, void *data, GLenum target, GLenum usage)
{
	if ( vbo_id_!=0 ) Delete();

	data_type_ = data_type;
	vec_dim_ = vec_dim;
	num_of_vectors_ = num_of_vectors;
	target_ = target;
	usage_ = usage;

	int byte_size_of_type = 0;
	switch (data_type)
	{
	case GL_BYTE:
	case GL_UNSIGNED_BYTE:
		byte_size_of_type = sizeof(GLbyte);
		break;
	case GL_SHORT:
	case GL_UNSIGNED_SHORT:
		byte_size_of_type = sizeof(GLshort);
		break;
	case GL_INT:
	case GL_UNSIGNED_INT:
		byte_size_of_type = sizeof(GLint);
		break;
	case GL_FLOAT:
		byte_size_of_type = sizeof(GLfloat);
		break;
	case GL_DOUBLE:
		byte_size_of_type = sizeof(GLdouble);
		break;
	default:
		std::cerr << "Undefined data type in GenAndBind()" << std::endl;
		exit(0);
		break;
	};

	glGenBuffers(1, &vbo_id_);
	glBindBuffer(target_, vbo_id_);
	glBufferData(target_, byte_size_of_type*vec_dim_*num_of_vectors_, data, usage_);
}

void
GL_VBO::Bind()
{
	if ( vbo_id_==0 ) return;
	glBindBuffer(target_, vbo_id_);
}



void 
GL_VBO::Update(void *data)
{
	int byte_size_of_type = 0;
	switch (data_type_)
	{
	case GL_BYTE:
	case GL_UNSIGNED_BYTE:
		byte_size_of_type = sizeof(GLbyte);
		break;
	case GL_SHORT:
	case GL_UNSIGNED_SHORT:
		byte_size_of_type = sizeof(GLshort);
		break;
	case GL_INT:
	case GL_UNSIGNED_INT:
		byte_size_of_type = sizeof(GLint);
		break;
	case GL_FLOAT:
		byte_size_of_type = sizeof(GLfloat);
		break;
	case GL_DOUBLE:
		byte_size_of_type = sizeof(GLdouble);
		break;
	default:
		std::cerr << "Undefined data type in GenAndBind()" << std::endl;
		exit(0);
		break;
	};
	Bind();
	glBufferSubData(target_, 0, byte_size_of_type*vec_dim_*num_of_vectors_, data);
}



void
GL_VBO::Delete()
{
	if ( vbo_id_ != 0 )
	{
		glDeleteBuffers(1, &vbo_id_);
	}
	vbo_id_= 0;
}










GL_VBOGroup::GL_VBOGroup(int num_vertices)
{
	ibo_id_ = 0;
	vbos_.resize(6, 0);

	num_vertices_ = num_vertices;
	num_indices_ = 0 ;
	drawing_mode_ = GL_TRIANGLES;
}

GL_VBOGroup::GL_VBOGroup()
{
	ibo_id_ = 0;
	vbos_.resize(6, 0);

	num_vertices_ = 0;
	num_indices_ = 0 ;
	drawing_mode_ = GL_TRIANGLES;
}

GL_VBOGroup::~GL_VBOGroup()
{
	Delete();
}


void
GL_VBOGroup::Delete()
{
	if ( ibo_id_ > 0 )
		glDeleteBuffers(1, &ibo_id_);

	ibo_id_ = 0;

	for ( unsigned int i=0; i<vbos_.size(); i++ )
	{
		if ( vbos_[i] != 0 ) delete vbos_[i];
		vbos_[i] = 0;
	}

	

	num_vertices_ = 0;
	num_indices_ = 0 ;
}

void
GL_VBOGroup::Init(int num_vertices)
{
	num_vertices_ = num_vertices;
}


void
GL_VBOGroup::SetByMesh(const mg::Mesh &mesh)
{
	Delete();

	mg::Mesh m(mesh);
	if ( m.mesh_type() == mg::Mesh::MT_POLYGONS )
		m.Triangulate();
	//m.UpdateNormalVectorsBasedOnVertex();
	

	if ( m.num_vertices() == 0 ) return;
	if ( m.mesh_type() == mg::Mesh::MT_TRIANGLES && m.num_faces() == 0 ) return;

	// vertices in homogeneous coordinate
	double *vertex_data = new double[m.num_vertices()*4];

	// normals
	double *normal_data = new double[m.num_vertices()*3];

	// tex_coord_2ds
	double *tex_coord_2ds_data = new double[m.num_vertices()*2];

	// colors
	double *color_data = new double[m.num_vertices()*4];


	// Indices of Triangles
	unsigned int *index_data = 0;
	if ( m.mesh_type() == mg::Mesh::MT_POINTS )    
	{
		index_data = new unsigned int[m.num_faces()*1];
	}
	else if ( m.mesh_type() == mg::Mesh::MT_LINES )     
	{
		index_data = new unsigned int[m.num_faces()*2];
	}
	else// if ( m.mesh_type() == mg::Mesh::MT_TRIANGLES ) 
	{
		index_data = new unsigned int[m.num_faces()*3];
	}

	// Bone
	int *bone_id_data = new int[m.num_vertices()*4];
	double *bone_weight_data = new double[m.num_vertices()*4];

	for ( int i=0; i<m.num_faces(); i++ )
	{
		for ( int j=0; j<m.face_size(i); j++ )
		{
			int vertex_id = m.face_vertex_ids(i)[j];

			vertex_data[vertex_id*4+0] = m.vertex(vertex_id)[0];
			vertex_data[vertex_id*4+1] = m.vertex(vertex_id)[1];
			vertex_data[vertex_id*4+2] = m.vertex(vertex_id)[2];
			vertex_data[vertex_id*4+3] = 1.0;

			if ( m.num_normals() > 0 )
			{
				int normal_id = m.face_normal_ids(i)[j];
				normal_data[vertex_id*3+0] = m.normal(normal_id)[0];
				normal_data[vertex_id*3+1] = m.normal(normal_id)[1];
				normal_data[vertex_id*3+2] = m.normal(normal_id)[2];
			}

			if ( m.num_uvs() > 0 )
			{
				int tex_coord_id = m.face_uv_ids(i)[j];
				tex_coord_2ds_data[vertex_id*2+0] = m.uv(tex_coord_id).first;
				tex_coord_2ds_data[vertex_id*2+1] = m.uv(tex_coord_id).second;
			}

			if ( m.num_colors() > 0 )
			{
				int color_id = m.face_color_ids(i)[j];
				color_data[vertex_id*4+0] = m.color(color_id)[0];
				color_data[vertex_id*4+1] = m.color(color_id)[1];
				color_data[vertex_id*4+2] = m.color(color_id)[2];
				color_data[vertex_id*4+3] = m.color(color_id)[3];
			}


			if ( m.has_bone() )
			{
				bone_id_data[vertex_id*4+0] = m.bone_ids(vertex_id)[0];
				bone_id_data[vertex_id*4+1] = m.bone_ids(vertex_id)[1];
				bone_id_data[vertex_id*4+2] = m.bone_ids(vertex_id)[2];
				bone_id_data[vertex_id*4+3] = m.bone_ids(vertex_id)[3];

				bone_weight_data[vertex_id*4+0] = m.bone_weights(vertex_id)[0];
				bone_weight_data[vertex_id*4+1] = m.bone_weights(vertex_id)[1];
				bone_weight_data[vertex_id*4+2] = m.bone_weights(vertex_id)[2];
				bone_weight_data[vertex_id*4+3] = m.bone_weights(vertex_id)[3];
			}
		}

		if ( m.mesh_type() == mg::Mesh::MT_POINTS )    
		{
			index_data[i] = m.face_vertex_ids(i)[0];
		}
		else if ( m.mesh_type() == mg::Mesh::MT_LINES )     
		{
			index_data[i*2+0] = m.face_vertex_ids(i)[0];
			index_data[i*2+1] = m.face_vertex_ids(i)[1];
		}
		else// if ( m.mesh_type() == mg::Mesh::MT_TRIANGLES ) 
		{
			index_data[i*3+0] = m.face_vertex_ids(i)[0];
			index_data[i*3+1] = m.face_vertex_ids(i)[1];
			index_data[i*3+2] = m.face_vertex_ids(i)[2];
		}
		
		
	}


	//CreateAndBindVertexArrayObject(m.num_vertices());
	Init(m.num_vertices());
	
	if ( m.num_vertices() > 0 )
		SetVBO(VERTEX_VBO, GL_DOUBLE, 4, vertex_data);
	
	if ( m.num_normals() > 0 )
		SetVBO(NORMAL_VBO, GL_DOUBLE, 3, normal_data);

	if ( m.num_uvs() > 0 )
		SetVBO(TEX_COORD_VBO, GL_DOUBLE, 2, tex_coord_2ds_data);

	if ( m.num_colors() > 0 )
		SetVBO(COLOR_VBO, GL_DOUBLE, 4, color_data);

	if ( m.num_faces() > 0 )
	{

		if ( m.mesh_type() == mg::Mesh::MT_POINTS )    
		{
			SetElementIndices(m.num_faces()*1, index_data);
		}
		else if ( m.mesh_type() == mg::Mesh::MT_LINES )     
		{
			SetElementIndices(m.num_faces()*2, index_data);
		}
		else// if ( m.mesh_type() == mg::Mesh::MT_TRIANGLES ) 
		{
			SetElementIndices(m.num_faces()*3, index_data);
		}
	}

	if ( m.has_bone() )
	{
		SetVBO(BONE_ID_VBO, GL_INT, 4, bone_id_data);
		SetVBO(BONE_WEIGHT_VBO, GL_DOUBLE, 4, bone_weight_data);
	}

	if (mesh.mesh_type() == Mesh::MT_POINTS)
	{
		drawing_mode(GL_POINTS);
	}
	else if (mesh.mesh_type() == Mesh::MT_LINES)
	{
		drawing_mode(GL_LINES);
	}
	else
		drawing_mode(GL_TRIANGLES);
		

	delete[] vertex_data;
	delete[] normal_data;
	delete[] tex_coord_2ds_data;
	delete[] color_data;
	delete[] index_data;
	delete[] bone_id_data;
	delete[] bone_weight_data;
}


void
GL_VBOGroup::UpdateVerticesByMesh(const mg::Mesh &mesh)
{
	mg::Mesh m(mesh);
	if ( m.mesh_type() == mg::Mesh::MT_POLYGONS )
		m.Triangulate();
	//m.UpdateNormalVectorsBasedOnVertex();


	if ( m.num_vertices() == 0 ) return;
	if ( m.mesh_type() == mg::Mesh::MT_TRIANGLES && m.num_faces() == 0 ) return;

	// vertices in homogeneous coordinate
	double *vertex_data = new double[m.num_vertices()*4];

	// normals
	double *normal_data = new double[m.num_vertices()*3];

	// tex_coord_2ds
	double *tex_coord_2ds_data = new double[m.num_vertices()*2];

	// colors
	double *color_data = new double[m.num_vertices()*4];

	// Indices of Triangles
	unsigned int *index_data = 0;
	if ( m.mesh_type() == mg::Mesh::MT_POINTS )    
	{
		index_data = new unsigned int[m.num_faces()*1];
	}
	else if ( m.mesh_type() == mg::Mesh::MT_LINES )     
	{
		index_data = new unsigned int[m.num_faces()*2];
	}
	else// if ( m.mesh_type() == mg::Mesh::MT_TRIANGLES ) 
	{
		index_data = new unsigned int[m.num_faces()*3];
	}

	// Bone
	int *bone_id_data = new int[m.num_vertices()*4];
	double *bone_weight_data = new double[m.num_vertices()*4];

	for ( int i=0; i<m.num_faces(); i++ )
	{
		for ( int j=0; j<m.face_size(i); j++ )
		{
			int vertex_id = m.face_vertex_ids(i)[j];

			vertex_data[vertex_id*4+0] = m.vertex(vertex_id)[0];
			vertex_data[vertex_id*4+1] = m.vertex(vertex_id)[1];
			vertex_data[vertex_id*4+2] = m.vertex(vertex_id)[2];
			vertex_data[vertex_id*4+3] = 1.0;

			if ( m.num_normals() > 0 )
			{
				int normal_id = m.face_normal_ids(i)[j];
				normal_data[vertex_id*3+0] = m.normal(normal_id)[0];
				normal_data[vertex_id*3+1] = m.normal(normal_id)[1];
				normal_data[vertex_id*3+2] = m.normal(normal_id)[2];
			}

			if ( m.num_uvs() > 0 )
			{
				int tex_coord_id = m.face_uv_ids(i)[j];
				tex_coord_2ds_data[vertex_id*2+0] = m.uv(tex_coord_id).first;
				tex_coord_2ds_data[vertex_id*2+1] = m.uv(tex_coord_id).second;
			}

			if ( m.num_colors() > 0 )
			{
				int color_id = m.face_color_ids(i)[j];
				color_data[vertex_id*4+0] = m.color(color_id)[0];
				color_data[vertex_id*4+1] = m.color(color_id)[1];
				color_data[vertex_id*4+2] = m.color(color_id)[2];
				color_data[vertex_id*4+3] = m.color(color_id)[3];
			}

			if ( m.has_bone() )
			{
				bone_id_data[vertex_id*4+0] = m.bone_ids(vertex_id)[0];
				bone_id_data[vertex_id*4+1] = m.bone_ids(vertex_id)[1];
				bone_id_data[vertex_id*4+2] = m.bone_ids(vertex_id)[2];
				bone_id_data[vertex_id*4+3] = m.bone_ids(vertex_id)[3];

				bone_weight_data[vertex_id*4+0] = m.bone_weights(vertex_id)[0];
				bone_weight_data[vertex_id*4+1] = m.bone_weights(vertex_id)[1];
				bone_weight_data[vertex_id*4+2] = m.bone_weights(vertex_id)[2];
				bone_weight_data[vertex_id*4+3] = m.bone_weights(vertex_id)[3];
			}
		}

		if ( m.mesh_type() == mg::Mesh::MT_POINTS )    
		{
			index_data[i] = m.face_vertex_ids(i)[0];
		}
		else if ( m.mesh_type() == mg::Mesh::MT_LINES )     
		{
			index_data[i*2+0] = m.face_vertex_ids(i)[0];
			index_data[i*2+1] = m.face_vertex_ids(i)[1];
		}
		else// if ( m.mesh_type() == mg::Mesh::MT_TRIANGLES ) 
		{
			index_data[i*3+0] = m.face_vertex_ids(i)[0];
			index_data[i*3+1] = m.face_vertex_ids(i)[1];
			index_data[i*3+2] = m.face_vertex_ids(i)[2];
		}

	}


	//CreateAndBindVertexArrayObject(m.num_vertices());

	Init(m.num_vertices());

	if ( m.num_vertices() > 0 )
		UpdateVBO(VERTEX_VBO, GL_DOUBLE, 4, vertex_data);

	if ( m.num_normals() > 0 )
		UpdateVBO(NORMAL_VBO, GL_DOUBLE, 3, normal_data);

	if ( m.num_uvs() > 0 )
		UpdateVBO(TEX_COORD_VBO, GL_DOUBLE, 2, tex_coord_2ds_data);

	if ( m.num_colors() > 0 )
		SetVBO(COLOR_VBO, GL_DOUBLE, 4, color_data);

	//if ( m.num_faces() > 0 )
		//SetElementIndices(m.num_faces()*3, index_data);

	if ( m.has_bone() )
	{
		UpdateVBO(BONE_ID_VBO, GL_INT, 4, bone_id_data);
		UpdateVBO(BONE_WEIGHT_VBO, GL_DOUBLE, 4, bone_weight_data);
	}

	if (mesh.mesh_type() == Mesh::MT_POINTS)
	{
		drawing_mode(GL_POINTS);
	}
	else if (mesh.mesh_type() == Mesh::MT_LINES)
	{
		drawing_mode(GL_LINES);
	}
	else
		drawing_mode(GL_TRIANGLES);


	delete[] vertex_data;
	delete[] normal_data;
	delete[] tex_coord_2ds_data;
	delete[] color_data;
	delete[] index_data;
	delete[] bone_id_data;
	delete[] bone_weight_data;
}






void
GL_VBOGroup::BindVBO()
{
	//ibo_id_ = 0;
	for ( unsigned int i=0; i<vbos_.size(); i++ )
	{
		if ( vbos_[i] )		vbos_[i]->Bind();
	}


	
}


void 
GL_VBOGroup::SetVBO(int content_type, GLenum data_type, int vec_dim, void *data)
{
	if ( num_vertices_ == 0 )
	{
		std::cerr << "num_vertices must be greater than 0 before SetVBO set (RenderableObjGL::SetVBO)" << std::endl;
		exit(0);
		return;
	}
	if ( content_type >= (int)vbos_.size() )
	{
		std::cerr << "Unknown VBO content type. (RenderableObjGL::SetVBO)" << std::endl;
		exit(0);
	}


	if ( vbos_[content_type] != 0 ) delete vbos_[content_type];
	vbos_[content_type] = new GL_VBO(data_type, vec_dim, num_vertices_, data);
	
}

void 
GL_VBOGroup::UpdateVBO(int content_type, GLenum data_type, int vec_dim, void *data)
{
	if ( num_vertices_ == 0 )
	{
		std::cerr << "num_vertices must be greater than 0 before SetVBO set (RenderableObjGL::SetVBO)" << std::endl;
		exit(0);
		return;
	}
	if ( content_type >= (int)vbos_.size() )
	{
		std::cerr << "Unknown VBO content type. (RenderableObjGL::SetVBO)" << std::endl;
		exit(0);
	}


	if ( vbos_[content_type] == 0 )
	{
		std::cerr << "vbos_[content_type] == 0. (RenderableObjGL::SetVBO)" << std::endl;
		exit(0);
	}
	vbos_[content_type]->Update(data);

}



void GL_VBOGroup::SetElementIndices(int num_indices, unsigned int *data)
{
	// if ( shader_ == 0 ) return;

	num_indices_ = num_indices;

	if ( ibo_id_ > 0 )
		glDeleteBuffers(1, &ibo_id_);

	ibo_id_ = 0;
	// glBindVertexArray(vao_);

	glGenBuffers(1, &ibo_id_);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo_id_);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, num_indices_*sizeof(unsigned int), data, GL_STATIC_DRAW);
}


static int g_num_vertices;
static cml::vector3f g_last_normal;
static cml::vector2f g_last_uv;
static cml::vector4f g_last_color;
static std::vector<cml::vector3f> g_tmp_normals;
static std::vector<cml::vector2f> g_tmp_uvs;
static std::vector<cml::vector4f> g_tmp_colors;
static std::vector<cml::vector3f> g_tmp_vertices;

void 
GL_VBOGroup::BeginVertex()
{
	Delete();
	g_num_vertices = 0;

	g_tmp_normals.clear();
	g_tmp_uvs.clear();
	g_tmp_colors.clear();
	g_tmp_vertices.clear();
}

void 
GL_VBOGroup::glNormal(cml::vector3f n)
{
	g_last_normal = n;
}
void 
GL_VBOGroup::glUV(cml::vector2f uv)
{
	g_last_uv = uv;
}

void 
GL_VBOGroup::glColor(cml::vector4f c)
{
	g_last_color = c;
}

void 
GL_VBOGroup::glVertex(cml::vector3f v)
{
	g_num_vertices++;
	g_tmp_normals.push_back(g_last_normal);
	g_tmp_uvs.push_back(g_last_uv);
	g_tmp_colors.push_back(g_last_color);
	g_tmp_vertices.push_back(v);
}


void
GL_VBOGroup::EndVertex()
{
	if ( g_num_vertices <= 0 ) return;

	Init(g_num_vertices);
	SetVBO(VERTEX_VBO, GL_FLOAT, 3, g_tmp_vertices.data());
	SetVBO(NORMAL_VBO, GL_FLOAT, 3, g_tmp_normals.data());
	SetVBO(COLOR_VBO, GL_FLOAT, 4, g_tmp_colors.data());
	SetVBO(TEX_COORD_VBO, GL_FLOAT, 2, g_tmp_uvs.data());
}










GL_VAO::GL_VAO(GL_VBOGroup* vbo_group)
{
	vao_id_ = 0;
	vbo_group_ = vbo_group;

	GenAndBind();
}

GL_VAO::~GL_VAO()
{
	Delete();
}

void
GL_VAO::Delete()
{
	if ( vao_id_ != 0 )
		glDeleteVertexArrays(1, &vao_id_);

	vao_id_ = 0;
}

void
GL_VAO::GenAndBind()
{
	
	glGenVertexArrays(1, &vao_id_);
	glBindVertexArray(vao_id_);

	UpdateVboGroupAttris();
/*
	if ( vbo != 0 )
	{
		vbo->Bind();
		glVertexAttribPointer(GL_ShaderProgram::V_POSITION_LOC, vbo->vec_dim(), vbo->data_type(), GL_FALSE, 0, NULL);
		glEnableVertexAttribArray(GL_ShaderProgram::V_POSITION_LOC);
	}

	vbo = vbo_group_->GetVBO(GL_VBOGroup::NORMAL_VBO);
	if ( vbo != 0 )
	{
		vbo->Bind();
		glVertexAttribPointer(GL_ShaderProgram::V_NORMAL_LOC, vbo->vec_dim(), vbo->data_type(), GL_FALSE, 0, NULL);
		glEnableVertexAttribArray(GL_ShaderProgram::V_NORMAL_LOC);
	}

	vbo = vbo_group_->GetVBO(GL_VBOGroup::TEX_COORD_VBO);
	if ( vbo != 0 )
	{
		vbo->Bind();
		glVertexAttribPointer(GL_ShaderProgram::V_TEX_COORD_LOC, vbo->vec_dim(), vbo->data_type(), GL_FALSE, 0, NULL);
		glEnableVertexAttribArray(GL_ShaderProgram::V_TEX_COORD_LOC);
	}

	vbo = vbo_group_->GetVBO(GL_VBOGroup::COLOR_VBO);
	if ( vbo != 0 )
	{
		vbo->Bind();
		glVertexAttribPointer(GL_ShaderProgram::V_COLOR_LOC, vbo->vec_dim(), vbo->data_type(), GL_FALSE, 0, NULL);
		glEnableVertexAttribArray(GL_ShaderProgram::V_COLOR_LOC);
	}
	
	vbo = vbo_group_->GetVBO(GL_VBOGroup::BONE_ID_VBO);
	if ( vbo != 0 )
	{
		vbo->Bind();
		glVertexAttribIPointer(GL_ShaderProgram::V_BONE_ID_LOC, vbo->vec_dim(), vbo->data_type(),  0, NULL);
		glEnableVertexAttribArray(GL_ShaderProgram::V_BONE_ID_LOC);
	}

	vbo = vbo_group_->GetVBO(GL_VBOGroup::BONE_WEIGHT_VBO);
	if ( vbo != 0 )
	{
		vbo->Bind();
		glVertexAttribPointer(GL_ShaderProgram::V_BONE_WEIGHT_LOC, vbo->vec_dim(), vbo->data_type(), GL_FALSE, 0, NULL);
		glEnableVertexAttribArray(GL_ShaderProgram::V_BONE_WEIGHT_LOC);
	}*/
}


void
GL_VAO::Bind()
{
	glBindVertexArray(vao_id_);
	//if ( vbo_group_ != 0) vbo_group_->BindVBO();
}

void
GL_VAO::UpdateVboGroupAttris()
{
	GL_VBO *vbo = vbo_group_->GetVBO(GL_VBOGroup::VERTEX_VBO);
	if (vbo != 0)
	{
		vbo->Bind();
		glVertexAttribPointer(GL_ShaderProgram::V_POSITION_LOC, vbo->vec_dim(), vbo->data_type(), GL_FALSE, 0, NULL);
		glEnableVertexAttribArray(GL_ShaderProgram::V_POSITION_LOC);
	}
	else
	{
		glDisableVertexAttribArray(GL_ShaderProgram::V_POSITION_LOC);
	}

	vbo = vbo_group_->GetVBO(GL_VBOGroup::NORMAL_VBO);
	if (vbo != 0)
	{
		vbo->Bind();
		glVertexAttribPointer(GL_ShaderProgram::V_NORMAL_LOC, vbo->vec_dim(), vbo->data_type(), GL_FALSE, 0, NULL);
		glEnableVertexAttribArray(GL_ShaderProgram::V_NORMAL_LOC);
	}
	else
	{
		glDisableVertexAttribArray(GL_ShaderProgram::V_POSITION_LOC);
	}

	vbo = vbo_group_->GetVBO(GL_VBOGroup::TEX_COORD_VBO);
	if (vbo != 0)
	{
		vbo->Bind();
		glVertexAttribPointer(GL_ShaderProgram::V_TEX_COORD_LOC, vbo->vec_dim(), vbo->data_type(), GL_FALSE, 0, NULL);
		glEnableVertexAttribArray(GL_ShaderProgram::V_TEX_COORD_LOC);
	}
	else
	{
		glDisableVertexAttribArray(GL_ShaderProgram::V_TEX_COORD_LOC);
	}

	vbo = vbo_group_->GetVBO(GL_VBOGroup::COLOR_VBO);
	if (vbo != 0)
	{
		vbo->Bind();
		glVertexAttribPointer(GL_ShaderProgram::V_COLOR_LOC, vbo->vec_dim(), vbo->data_type(), GL_FALSE, 0, NULL);
		glEnableVertexAttribArray(GL_ShaderProgram::V_COLOR_LOC);
	}
	else
	{
		glDisableVertexAttribArray(GL_ShaderProgram::V_COLOR_LOC);
	}

	vbo = vbo_group_->GetVBO(GL_VBOGroup::BONE_ID_VBO);
	if (vbo != 0)
	{
		vbo->Bind();
		glVertexAttribIPointer(GL_ShaderProgram::V_BONE_ID_LOC, vbo->vec_dim(), vbo->data_type(), 0, NULL);
		glEnableVertexAttribArray(GL_ShaderProgram::V_BONE_ID_LOC);
	}
	else
	{
		glDisableVertexAttribArray(GL_ShaderProgram::V_BONE_ID_LOC);
	}

	vbo = vbo_group_->GetVBO(GL_VBOGroup::BONE_WEIGHT_VBO);
	if (vbo != 0)
	{
		vbo->Bind();
		glVertexAttribPointer(GL_ShaderProgram::V_BONE_WEIGHT_LOC, vbo->vec_dim(), vbo->data_type(), GL_FALSE, 0, NULL);
		glEnableVertexAttribArray(GL_ShaderProgram::V_BONE_WEIGHT_LOC);
	}
	else
	{
		glDisableVertexAttribArray(GL_ShaderProgram::V_BONE_WEIGHT_LOC);
	}
}



void
GL_VAO::Draw() const
{
	if ( vbo_group_ == 0 ) return;

	glBindVertexArray(vao_id_);

	//if ( shader_->flag_skinning() )
	//{
	//	for ( std::set<int>::iterator iter = using_bone_matrix_ids_.begin();
	//		iter != using_bone_matrix_ids_.end();
	//		iter++ )
	//	{
	//		int i = *iter;
	//		cml::matrix44f m_and_offset = GetBoneMatrix(i) * cml::inverse( GetBoneOffsetMatrix(i) );
	//		shader_->SetBoneMatrix(i, ((cml::matrix44f)m_and_offset).data() ); 
	//	}
	//	//updated_bone_matrix_ids_.clear();
	//}

	if ( vbo_group_->ibo_id() != 0 )
	{
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbo_group_->ibo_id());
		glDrawElements(vbo_group_->drawing_mode(), vbo_group_->num_indices(), GL_UNSIGNED_INT, NULL);
	}
	else
	{
		glDrawArrays(vbo_group_->drawing_mode(), 0, vbo_group_->num_vertices());
	}
}




};








