
#include "BaseLib/GL4U/GL_ResourceManager.h"
#include "BaseLib/GL4U/GL_Mesh.h"
#include "BaseLib/GL4U/GL_SceneGraph.h"

#include "assimp/scene.h"
#include "assimp/Importer.hpp"
#include "assimp/postprocess.h"
#include "assimp/DefaultLogger.hpp"
#include "assimp/LogStream.hpp"
#include "IL/il.h"

namespace mg
{
void
GL_VBOGroup::SetByAssimp(const std::string filename)
{
	std::vector< std::pair<std::string, cml::matrix44d> > out_bone_name_and_mat;
	SetByAssimp(filename, out_bone_name_and_mat);
}

void
GL_VBOGroup::SetByAssimp(const std::string filename,  std::vector< std::pair<std::string, cml::matrix44d> > &out_bone_name_and_mat)
{
	std::map<std::string, int> empty_bonename_to_pmjoint;
	SetByAssimp(filename, empty_bonename_to_pmjoint, out_bone_name_and_mat);
}

void
GL_VBOGroup::SetByAssimp(const std::string filename,  const std::map<std::string, int> &in_bonename_to_pmjoint)
{
	std::vector< std::pair<std::string, cml::matrix44d> > out_bone_name_and_mat;
	SetByAssimp(filename, in_bonename_to_pmjoint, out_bone_name_and_mat);
}


void
GL_VBOGroup::SetByAssimp(const std::string filename, 
					const std::map<std::string, int> &in_bonename_to_pmjoint, 
					std::vector< std::pair<std::string, cml::matrix44d> > &out_bone_name_and_mat)
{
	Delete();

	//Assimp::Logger::LogSeverity severity = Assimp::Logger::NORMAL;
	Assimp::Logger::LogSeverity severity = Assimp::Logger::VERBOSE;

	// Create a logger instance for Console Output
	Assimp::DefaultLogger::create("",severity, aiDefaultLogStream_STDOUT);

	// Create a logger instance for File Output (found in project folder or near .exe)
	Assimp::DefaultLogger::create("assimp_log.txt",severity, aiDefaultLogStream_FILE);

	// Now I am ready for logging my stuff
	Assimp::DefaultLogger::get()->info("this is my info-call");


	Assimp::Importer ai_importer;

	const aiScene *ai_scene_ = ai_importer.ReadFile(filename, aiProcessPreset_TargetRealtime_Quality|aiProcess_Triangulate);

	// Kill it after the work is done
	Assimp::DefaultLogger::kill();

	//ai_scene_->mMaterials

	for ( unsigned int i=0; i<ai_scene_->mNumMeshes; i++ )
	{
		const aiMesh *mesh = ai_scene_->mMeshes[i];
		if ( mesh->HasBones() )
		{
			Init(mesh->mNumVertices);

			// vertices
			SetVBO(VERTEX_VBO, GL_FLOAT, 3, (void*)mesh->mVertices);
			
			// normals
			SetVBO(NORMAL_VBO, GL_FLOAT, 3, (void*)mesh->mNormals);
			

			// uvs
			{
				if ( mesh->GetNumUVChannels() >= 1 )
				{
					if ( mesh->mNumUVComponents[0] == 2 )
					{
						SetVBO(TEX_COORD_VBO, GL_FLOAT, 3, (void*)mesh->mTextureCoords[0]);
					}
				}
			}

			// color
			{
				if ( mesh->GetNumColorChannels() >= 1 )
				{
					SetVBO(COLOR_VBO, GL_FLOAT, 4, (void*)mesh->mColors[0]);
				}
			}

			

			// Triangle indices
			{
				unsigned int *tri_indices = new unsigned int[mesh->mNumFaces * 3];

				for ( unsigned int i=0; i<mesh->mNumFaces; i++ )
				{
					if ( mesh->mFaces[i].mNumIndices == 1 ) 
					{
						tri_indices[i * 3 + 0] = mesh->mFaces[i].mIndices[0];
						tri_indices[i * 3 + 1] = mesh->mFaces[i].mIndices[0];
						tri_indices[i * 3 + 2] = mesh->mFaces[i].mIndices[0];
					}
					else if (mesh->mFaces[i].mNumIndices == 2)
					{
						tri_indices[i * 3 + 0] = mesh->mFaces[i].mIndices[0];
						tri_indices[i * 3 + 1] = mesh->mFaces[i].mIndices[1];
						tri_indices[i * 3 + 2] = mesh->mFaces[i].mIndices[1];
					}
					else
					{
						tri_indices[i * 3 + 0] = mesh->mFaces[i].mIndices[0];
						tri_indices[i * 3 + 1] = mesh->mFaces[i].mIndices[1];
						tri_indices[i * 3 + 2] = mesh->mFaces[i].mIndices[2];
					}
				}
				SetElementIndices(mesh->mNumFaces*3, tri_indices);
				drawing_mode(GL_TRIANGLES);

				delete[] tri_indices;
			}

			// Return bone names and offset_matrices
			if ( mesh->HasBones() )
			{
				for ( unsigned int i=0; i<mesh->mNumBones; i++ )
				{
					std::pair<std::string, cml::matrix44d> name_and_mat;
					
					name_and_mat.first = std::string( mesh->mBones[i]->mName.C_Str() );

					aiNode *node = ai_scene_->mRootNode->FindNode( mesh->mBones[i]->mName );

					aiMatrix4x4 m = mesh->mBones[i]->mOffsetMatrix;
					m.Inverse();
					

					for ( int j=0; j<4; j++ )
					for ( int k=0; k<4; k++ )
					{
						name_and_mat.second.set_basis_element(j, k
							, m[k][j]);
					}

					

					out_bone_name_and_mat.push_back(name_and_mat);
				}
			}

			
			// Weight
			if ( mesh->HasBones() )
			{
				// We assume that maximum 4 bones can be related for a vertex at most.

				int *nums_bones_per_vertex = new int[mesh->mNumVertices];
				int *bone_ids = new int[4*mesh->mNumVertices];
				double *bone_weights = new double[4*mesh->mNumVertices];

				// initialize
				for ( unsigned int i=0; i<mesh->mNumVertices; i++ )
				{
					nums_bones_per_vertex[i] = 0;
					bone_ids[i*4+0] = bone_ids[i*4+1] = bone_ids[i*4+2] = bone_ids[i*4+3] = -1;
					bone_weights[i*4+0] = bone_weights[i*4+1] = bone_weights[i*4+2] = bone_weights[i*4+3] = 0.0f;
				}


				for ( unsigned int i=0; i<mesh->mNumBones; i++ )
				{
					for ( unsigned int j=0; j<mesh->mBones[i]->mNumWeights; j++ )
					{
						int vertex_id = mesh->mBones[i]->mWeights[j].mVertexId;
						float weight = mesh->mBones[i]->mWeights[j].mWeight;

						// If there are more than four joints related at one vertex,
						// we just skip the fifth and later one. 
						if ( nums_bones_per_vertex[vertex_id] == 4 )
						{
							std::cerr << "the number of bones for one vertex is more than 4." << std::endl;
							continue;
						}
						

						int bone_id = i;
						if ( in_bonename_to_pmjoint.find(mesh->mBones[i]->mName.C_Str()) != in_bonename_to_pmjoint.end() )
						{
							bone_id = in_bonename_to_pmjoint.find(mesh->mBones[i]->mName.C_Str())->second;
						}
						bone_ids[ vertex_id*4 + nums_bones_per_vertex[vertex_id] ] = bone_id;
						bone_weights[ vertex_id*4 + nums_bones_per_vertex[vertex_id] ] = weight;

						nums_bones_per_vertex[vertex_id]++;
					}
				
					
				}

				SetVBO(BONE_ID_VBO, GL_INT, 4, bone_ids);
				SetVBO(BONE_WEIGHT_VBO, GL_DOUBLE, 4, bone_weights);


				delete[] nums_bones_per_vertex;
				delete[] bone_ids;
				delete[] bone_weights;
			}

			break;
		}
	}
}




// images / texture
// static std::string modelpath;
// std::map<std::string, GLuint*> textureIdMap;	// map image filenames to textureIds
//  GLuint*		textureIds;	

std::string getBasePath(const std::string& path)
{
	size_t pos = path.find_last_of("\\/");
	return (std::string::npos == pos) ? "" : path.substr(0, pos + 1);
}

static std::string g_ai_file_base_path;
static std::vector<GL_Material*> g_tmp_materials;


mg::GL_Texture* CreateGLTextures(std::string filename, std::string tex_name="")
{
	ILboolean success;

	mg::GL_Texture *out_gl_tex = nullptr;

	/* Before calling ilInit() version should be checked. */
	if (ilGetInteger(IL_VERSION_NUM) < IL_VERSION)
	{
		/// wrong DevIL version ///
		std::string err_msg = "Wrong DevIL version. Old devil.dll in system32/SysWow64?";
		char* cErr_msg = (char *)err_msg.c_str();
		return false;
	}

	ilInit(); /* Initialization of DevIL */

	
	/* array with DevIL image IDs */
	ILuint imageId;

	/* generate DevIL Image IDs */
	ilGenImages(1, &imageId); /* Generation of numTextures image names */

	ilBindImage(imageId); /* Binding of DevIL image name */
	
	success = ilLoadImage(filename.c_str());

	if (success) /* If no error occured: */
	{
		success = ilConvertImage(IL_RGB, IL_UNSIGNED_BYTE); /* Convert every colour component into
		unsigned byte. If your image contains alpha channel you can replace IL_RGB with IL_RGBA */
		if (!success)
		{
			/* Error occured */
			// MessageBox(NULL, ("Couldn't convert image" + fileloc).c_str(), "ERROR", MB_OK | MB_ICONEXCLAMATION);
			std::cerr << "Couldn't convert image " + filename <<std::endl;
			return nullptr;
		}
		else
		{
			////glGenTextures(numTextures, &textureIds[i]); /* Texture name generation */
			//glBindTexture(GL_TEXTURE_2D, textureIds[i]); /* Binding of texture name */
			////redefine standard texture values
			//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR); /* We will use linear
			//interpolation for magnification filter */
			//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR); /* We will use linear
			//interpolation for minifying filter */
			//glTexImage2D(GL_TEXTURE_2D, 0, ilGetInteger(IL_IMAGE_BPP), ilGetInteger(IL_IMAGE_WIDTH),
			//	ilGetInteger(IL_IMAGE_HEIGHT), 0, ilGetInteger(IL_IMAGE_FORMAT), GL_UNSIGNED_BYTE,
			//	ilGetData()); /* Texture specification */

			out_gl_tex = mg::GL_ResourceManager::singleton()->CreateTexture(tex_name);// new mg::GL_Texture;

			out_gl_tex->Gen2DAndBind(ilGetInteger(IL_IMAGE_WIDTH), ilGetInteger(IL_IMAGE_HEIGHT));
			out_gl_tex->SetData(ilGetData());

		}
	}
	else
	{
		/* Error occured */
		// MessageBox(NULL, ("Couldn't load Image: " + fileloc).c_str(), "ERROR", MB_OK | MB_ICONEXCLAMATION);
	}


	ilDeleteImages(1, &imageId); /* Because we have already copied image data into texture data
	we can release memory used by image. */

	//Cleanup
	
	//return success;
	return out_gl_tex;
}


// Can't send color down as a pointer to aiColor4D because AI colors are ABGR.
static void Color4f(const aiColor4D *color)
{
	glColor4f(color->r, color->g, color->b, color->a);
}

static void set_float4(float f[4], float a, float b, float c, float d)
{
	f[0] = a;
	f[1] = b;
	f[2] = c;
	f[3] = d;
}

static void color4_to_float4(const aiColor4D *c, float f[4])
{
	f[0] = c->r;
	f[1] = c->g;
	f[2] = c->b;
	f[3] = c->a;
}


static mg::GL_Material* CreateGlMaterial(const aiMaterial *mtl, std::string name = "")
{
	//mg::GL_Material *out_gl_material = new mg::GL_Material;
	mg::GL_Material *out_gl_material = mg::GL_ResourceManager::singleton()->CreateMaterial(name);

	float c[4];

	// GLenum fill_mode;
	int ret1, ret2;
	aiColor4D diffuse;
	aiColor4D specular;
	aiColor4D ambient;
	aiColor4D emission;
	float shininess, strength;
	// int two_sided;
	// int wireframe;
	unsigned int max;	// changed: to unsigned

	int texIndex = 0;
	aiString texPath;	//contains filename of texture
	aiTextureMapMode tex_wrap_mode;

	if(AI_SUCCESS == mtl->GetTexture(aiTextureType_DIFFUSE, texIndex, &texPath, 0, 0, 0, 0, &tex_wrap_mode))
	{
		//bind texture
		// unsigned int texId = *textureIdMap[texPath.data];
		// glBindTexture(GL_TEXTURE_2D, texId);
		GL_Texture *tex = CreateGLTextures(g_ai_file_base_path + texPath.C_Str());
		
		if ( tex_wrap_mode == aiTextureMapMode_Wrap )
			tex->gl_texture_wrap(GL_REPEAT);
		else if ( tex_wrap_mode == aiTextureMapMode_Mirror )
			tex->gl_texture_wrap(GL_MIRRORED_REPEAT);
		else 
			tex->gl_texture_wrap(GL_CLAMP_TO_EDGE);

		if ( tex != nullptr ) 
		{
			out_gl_material->AddTexture(tex);
			out_gl_material->flag_texture(true);
		}
	}

	//set_float4(c, 0.8f, 0.8f, 0.8f, 1.0f);
	if(AI_SUCCESS == aiGetMaterialColor(mtl, AI_MATKEY_COLOR_DIFFUSE, &diffuse))
	{
		color4_to_float4(&diffuse, c);
		out_gl_material->diffuse_color(c[0], c[1], c[2], c[3]);
	}

	//set_float4(c, 0.0f, 0.0f, 0.0f, 1.0f);
	if(AI_SUCCESS == aiGetMaterialColor(mtl, AI_MATKEY_COLOR_SPECULAR, &specular))
	{
		color4_to_float4(&specular, c);
		out_gl_material->specular_color(c[0], c[1], c[2], c[3]);
	}

	//set_float4(c, 0.2f, 0.2f, 0.2f, 1.0f);
	if(AI_SUCCESS == aiGetMaterialColor(mtl, AI_MATKEY_COLOR_AMBIENT, &ambient))
	{
		color4_to_float4(&ambient, c);
		out_gl_material->ambient_color(c[0], c[1], c[2], c[3]);
	}

	//set_float4(c, 0.0f, 0.0f, 0.0f, 1.0f);
	if(AI_SUCCESS == aiGetMaterialColor(mtl, AI_MATKEY_COLOR_EMISSIVE, &emission))
	{
		color4_to_float4(&emission, c);
		out_gl_material->emission_color(c[0], c[1], c[2], c[3]);
	}

	max = 1;
	ret1 = aiGetMaterialFloatArray(mtl, AI_MATKEY_SHININESS, &shininess, &max);
	if( ret1 == AI_SUCCESS )
		out_gl_material->shininess(shininess);

	max = 1;
	ret2 = aiGetMaterialFloatArray(mtl, AI_MATKEY_SHININESS_STRENGTH, &strength, &max);
	if( ret2 == AI_SUCCESS )
		out_gl_material->shininess_strength(strength);

	return out_gl_material;
	
/*

	max = 1;
	if(AI_SUCCESS == aiGetMaterialIntegerArray(mtl, AI_MATKEY_ENABLE_WIREFRAME, &wireframe, &max))
		fill_mode = wireframe ? GL_LINE : GL_FILL;
	else
		fill_mode = GL_FILL;
	glPolygonMode(GL_FRONT_AND_BACK, fill_mode);

	max = 1;
	if((AI_SUCCESS == aiGetMaterialIntegerArray(mtl, AI_MATKEY_TWOSIDED, &two_sided, &max)) && two_sided)
		glEnable(GL_CULL_FACE);
	else
		glDisable(GL_CULL_FACE);*/
}

static GL_RenderableObj* CreateRenderableObj_old(const aiMesh *ai_mesh, std::string obj_name="")
{
	std::string vbogroup_name = GL_ResourceManager::singleton()->GenUniqueName(obj_name+"_VBOGroup");
	mg::GL_VBOGroup* vbo_group = mg::GL_ResourceManager::singleton()->CreateVBOGroup(vbogroup_name);

	vbo_group->Init(ai_mesh->mNumVertices);

	// vertices
	vbo_group->SetVBO(GL_VBOGroup::VERTEX_VBO, GL_FLOAT, 3, (void*)ai_mesh->mVertices);

	// normals
	vbo_group->SetVBO(GL_VBOGroup::NORMAL_VBO, GL_FLOAT, 3, (void*)ai_mesh->mNormals);


	// tex_coords
	{
		if (ai_mesh->GetNumUVChannels() >= 1)
		{
			if (ai_mesh->mNumUVComponents[0] == 2)
			{
				// transform 
				for (unsigned int t_i = 0; t_i<ai_mesh->mNumVertices; t_i++)
				{
					ai_mesh->mTextureCoords[0][t_i][1] = 1 - ai_mesh->mTextureCoords[0][t_i][1];
				}
				vbo_group->SetVBO(GL_VBOGroup::TEX_COORD_VBO, GL_FLOAT, 3, (void*)ai_mesh->mTextureCoords[0]);
			}
		}
	}

	// color
	{
		if (ai_mesh->GetNumColorChannels() >= 1)
		{
			vbo_group->SetVBO(GL_VBOGroup::COLOR_VBO, GL_FLOAT, 4, (void*)ai_mesh->mColors[0]);
		}
	}


	// Point indices
	if ( ai_mesh->mPrimitiveTypes == aiPrimitiveType_POINT )
	{
		unsigned int *point_indices = new unsigned int[ai_mesh->mNumFaces * 1];

		for (unsigned int i = 0; i<ai_mesh->mNumFaces; i++)
		{
			if ( ai_mesh->mFaces[i].mNumIndices == 1 ) 
			{
				point_indices[i * 1 + 0] = ai_mesh->mFaces[i].mIndices[0];
			}
		}
		vbo_group->SetElementIndices(ai_mesh->mNumFaces * 1, point_indices);
		vbo_group->drawing_mode(GL_POINTS);

		delete[] point_indices;
	}
	// Line indices
	else if ( ai_mesh->mPrimitiveTypes == aiPrimitiveType_LINE )
	{
		unsigned int *line_indices = new unsigned int[ai_mesh->mNumFaces * 2];

		for (unsigned int i = 0; i<ai_mesh->mNumFaces; i++)
		{
			if ( ai_mesh->mFaces[i].mNumIndices == 1 ) 
			{
				line_indices[i * 2 + 0] = ai_mesh->mFaces[i].mIndices[0];
				line_indices[i * 2 + 1] = ai_mesh->mFaces[i].mIndices[0];
			}
			else if (ai_mesh->mFaces[i].mNumIndices == 2)
			{
				line_indices[i * 2 + 0] = ai_mesh->mFaces[i].mIndices[0];
				line_indices[i * 2 + 1] = ai_mesh->mFaces[i].mIndices[1];
			}
		}
		vbo_group->SetElementIndices(ai_mesh->mNumFaces * 2, line_indices);
		vbo_group->drawing_mode(GL_LINES);

		delete[] line_indices;
	}
	// Triangle indices
	else
	{
		unsigned int *tri_indices = new unsigned int[ai_mesh->mNumFaces * 3];

		for (unsigned int i = 0; i<ai_mesh->mNumFaces; i++)
		{
			if ( ai_mesh->mFaces[i].mNumIndices == 1 ) 
			{
				tri_indices[i * 3 + 0] = ai_mesh->mFaces[i].mIndices[0];
				tri_indices[i * 3 + 1] = ai_mesh->mFaces[i].mIndices[0];
				tri_indices[i * 3 + 2] = ai_mesh->mFaces[i].mIndices[0];
			}
			else if (ai_mesh->mFaces[i].mNumIndices == 2)
			{
				tri_indices[i * 3 + 0] = ai_mesh->mFaces[i].mIndices[0];
				tri_indices[i * 3 + 1] = ai_mesh->mFaces[i].mIndices[1];
				tri_indices[i * 3 + 2] = ai_mesh->mFaces[i].mIndices[1];
			}
			else
			{
				tri_indices[i * 3 + 0] = ai_mesh->mFaces[i].mIndices[0];
				tri_indices[i * 3 + 1] = ai_mesh->mFaces[i].mIndices[1];
				tri_indices[i * 3 + 2] = ai_mesh->mFaces[i].mIndices[2];
			}
		}
		vbo_group->SetElementIndices(ai_mesh->mNumFaces * 3, tri_indices);
		vbo_group->drawing_mode(GL_TRIANGLES);

		delete[] tri_indices;
	}

	// Weight
	if (ai_mesh->HasBones())
	{
		// We assume that maximum 4 bones can be related for a vertex at most.

		int *nums_bones_per_vertex = new int[ai_mesh->mNumVertices];
		int *bone_ids = new int[4 * ai_mesh->mNumVertices];
		double *bone_weights = new double[4 * ai_mesh->mNumVertices];

		// initialize
		for (unsigned int i = 0; i<ai_mesh->mNumVertices; i++)
		{
			nums_bones_per_vertex[i] = 0;
			bone_ids[i * 4 + 0] = bone_ids[i * 4 + 1] = bone_ids[i * 4 + 2] = bone_ids[i * 4 + 3] = -1;
			bone_weights[i * 4 + 0] = bone_weights[i * 4 + 1] = bone_weights[i * 4 + 2] = bone_weights[i * 4 + 3] = 0.0f;
		}


		for (unsigned int i = 0; i<ai_mesh->mNumBones; i++)
		{
			for (unsigned int j = 0; j<ai_mesh->mBones[i]->mNumWeights; j++)
			{
				int vertex_id = ai_mesh->mBones[i]->mWeights[j].mVertexId;
				float weight = ai_mesh->mBones[i]->mWeights[j].mWeight;

				// If there are more than four joints related at one vertex,
				// we just skip the fifth ane later one. 
				if (nums_bones_per_vertex[vertex_id] == 4)
				{
					std::cerr << "static GL_RenderableObj* CreateRenderableObj(): Warning More than 4 bones are related to one vertex." << std::endl;
					continue;
				}


				int bone_id = i;
				bone_ids[vertex_id * 4 + nums_bones_per_vertex[vertex_id]] = bone_id;
				bone_weights[vertex_id * 4 + nums_bones_per_vertex[vertex_id]] = weight;

				nums_bones_per_vertex[vertex_id]++;
			}


		}

		vbo_group->SetVBO(GL_VBOGroup::BONE_ID_VBO, GL_INT, 4, bone_ids);
		vbo_group->SetVBO(GL_VBOGroup::BONE_WEIGHT_VBO, GL_DOUBLE, 4, bone_weights);


		delete[] nums_bones_per_vertex;
		delete[] bone_ids;
		delete[] bone_weights;
	}
	

	// Material
	GL_Material *gl_material = nullptr;
	{
		if (ai_mesh->mMaterialIndex >= 0 && ai_mesh->mMaterialIndex < g_tmp_materials.size() )
		{
			gl_material = g_tmp_materials[ai_mesh->mMaterialIndex];
		}
		else
		{
			// aiMaterial *mat = ai_scene_->mMaterials[mesh->mMaterialIndex];
			// gl_material = CreateGlMaterial(mat);

			//???
			std::cerr << "static GL_RenderableObj* CreateRenderableObj(): No available material in g_tmp_materials" << std::endl;
		}
	}


	GL_RenderableObj *r_obj = GL_ResourceManager::singleton()->CreateRenderableObj(obj_name, vbo_group, gl_material);
	if ( gl_material ) r_obj->flag_material(true);


	// Set bone names and offset_matrices
	if ( ai_mesh->HasBones() )
	{
		r_obj->flag_skinning(true);
		for (unsigned int i = 0; i<ai_mesh->mNumBones; i++)
		{
			std::pair<std::string, cml::matrix44d> name_and_mat;

			name_and_mat.first = std::string(ai_mesh->mBones[i]->mName.C_Str());

			// aiNode *node = ai_scene_->mRootNode->FindNode(ai_mesh->mBones[i]->mName);

			aiMatrix4x4 m = ai_mesh->mBones[i]->mOffsetMatrix;
			m.Inverse();


			for (int j = 0; j<4; j++)
				for (int k = 0; k<4; k++)
				{
					name_and_mat.second.set_basis_element(j, k
						, m[k][j]);
				}

			r_obj->SetBoneName(i, name_and_mat.first);
			r_obj->SetBoneOffsetMatrix(i, name_and_mat.second);
		}
	}

	return r_obj;
}


/**
Assimp sytem distinquishes bones by name. 
But I want give a unique integer id for each bone.
*/
std::map<std::string, int> g_bone_name_to_id; 
int g_count_bone = 0;

static int GetBoneId(std::string bone_name)
{
	if ( g_bone_name_to_id.find(bone_name) != g_bone_name_to_id.end() )
		return g_bone_name_to_id[bone_name];

	g_bone_name_to_id[bone_name] = g_count_bone;
	g_count_bone++;

	return g_bone_name_to_id[bone_name];
}

static GL_RenderableObj* CreateRenderableObj(const aiMesh *ai_mesh, std::string obj_name="")
{
	std::string mesh_name = ai_mesh->mName.C_Str();
	
	if ( mesh_name.empty() )
	{
		mesh_name = obj_name+"_MESH";
	}

	GL_Mesh *mesh = GL_ResourceManager::singleton()->CreateMesh(mesh_name);

	if ( ai_mesh->mVertices==0 || ai_mesh->mNumVertices== 0)
	{
		std::cerr << "warning!! ai_mesh is empty! - CreateRenderableObj()" << std::endl;
		return GL_ResourceManager::singleton()->CreateRenderableObj("", mesh);
	}

	mesh->SetVertices(ai_mesh->mNumVertices);
	for ( int i=0; i<(int)ai_mesh->mNumVertices; i++ )
	{
		mesh->vertex(i, {ai_mesh->mVertices[i].x, ai_mesh->mVertices[i].y, ai_mesh->mVertices[i].z});
	}

	if ( ai_mesh->mNormals )
	{
		mesh->SetNormals(ai_mesh->mNumVertices);
		for ( int i=0; i<(int)ai_mesh->mNumVertices; i++ )
		{
			mesh->normal(i, {ai_mesh->mNormals[i].x, ai_mesh->mNormals[i].y, ai_mesh->mNormals[i].z});
		}
	}

	// tex_coords
	{
		if (ai_mesh->GetNumUVChannels() >= 1)
		{
			if (ai_mesh->mNumUVComponents[0] == 2)
			{
				mesh->SetUVs(ai_mesh->mNumVertices);
				// transform and Set
				for (unsigned int t_i = 0; t_i<ai_mesh->mNumVertices; t_i++)
				{
					ai_mesh->mTextureCoords[0][t_i][1] = 1 - ai_mesh->mTextureCoords[0][t_i][1];
					mesh->uv(t_i, ai_mesh->mTextureCoords[0][t_i][0], ai_mesh->mTextureCoords[0][t_i][1]);
				}
			}
		}
	}

	// colors
	if (ai_mesh->GetNumColorChannels() >= 1)
	{
		{
			// vbo_group->SetVBO(GL_VBOGroup::COLOR_VBO, GL_FLOAT, 4, (void*)ai_mesh->mColors[0]);
		}
		mesh->SetColors(ai_mesh->mNumVertices);
		for ( int i=0; i<(int)ai_mesh->mNumVertices; i++ )
		{
			mesh->color(i, {ai_mesh->mColors[0][i].r, ai_mesh->mColors[0][i].g, ai_mesh->mColors[0][i].b, ai_mesh->mColors[0][i].a});
		}
	}


	// Point indices
	if ( ai_mesh->mPrimitiveTypes == aiPrimitiveType_POINT )
	{
		mesh->SetNumFaces(ai_mesh->mNumFaces);
		for (unsigned int i = 0; i<ai_mesh->mNumFaces; i++)
		{
			int id0;
			if ( ai_mesh->mFaces[i].mNumIndices == 1 ) 
			{
				id0 = ai_mesh->mFaces[i].mIndices[0];

				mesh->SetFaceSize(i, 1);
				if ( mesh->num_vertices() > 0 ) mesh->SetFaceVertexIds(i, id0);
				if ( mesh->num_normals()  > 0 ) mesh->SetFaceNormalIds(i, id0);
				if ( mesh->num_uvs()      > 0 ) mesh->SetFaceUvIds    (i, id0);
				if ( mesh->num_colors()   > 0 ) mesh->SetFaceColorIds (i, id0);
			}
		}
		mesh->mesh_type(Mesh::MT_POINTS);
	}

	// Line indices
	else if ( ai_mesh->mPrimitiveTypes == aiPrimitiveType_LINE )
	{
		mesh->SetNumFaces(ai_mesh->mNumFaces);
		for (unsigned int i = 0; i<ai_mesh->mNumFaces; i++)
		{
			int id0, id1;
			if ( ai_mesh->mFaces[i].mNumIndices == 1 ) 
			{
				id0 = ai_mesh->mFaces[i].mIndices[0];
				id1 = ai_mesh->mFaces[i].mIndices[0];

			}
			else if (ai_mesh->mFaces[i].mNumIndices == 2)
			{
				id0 = ai_mesh->mFaces[i].mIndices[0];
				id1 = ai_mesh->mFaces[i].mIndices[1];
			}

			mesh->SetFaceSize(i, 2);
			if ( mesh->num_vertices() > 0 ) mesh->SetFaceVertexIds(i, id0, id1);
			if ( mesh->num_normals()  > 0 ) mesh->SetFaceNormalIds(i, id0, id1);
			if ( mesh->num_uvs()      > 0 ) mesh->SetFaceUvIds    (i, id0, id1);
			if ( mesh->num_colors()   > 0 ) mesh->SetFaceColorIds (i, id0, id1);

		}
		mesh->mesh_type(Mesh::MT_LINES);
	}

	// Triangle indices
	else
	{
		mesh->SetNumFaces(ai_mesh->mNumFaces);
		for (unsigned int i = 0; i<ai_mesh->mNumFaces; i++)
		{
			int id0, id1, id2;
			if ( ai_mesh->mFaces[i].mNumIndices == 1 ) 
			{
				id0 = ai_mesh->mFaces[i].mIndices[0];
				id1 = ai_mesh->mFaces[i].mIndices[0];
				id2 = ai_mesh->mFaces[i].mIndices[0];

			}
			else if (ai_mesh->mFaces[i].mNumIndices == 2)
			{
				id0 = ai_mesh->mFaces[i].mIndices[0];
				id1 = ai_mesh->mFaces[i].mIndices[1];
				id2 = ai_mesh->mFaces[i].mIndices[1];
			}
			else
			{
				id0 = ai_mesh->mFaces[i].mIndices[0];
				id1 = ai_mesh->mFaces[i].mIndices[1];
				id2 = ai_mesh->mFaces[i].mIndices[2];
			}

			mesh->SetFaceSize(i, 3);
			if ( mesh->num_vertices() > 0 ) mesh->SetFaceVertexIds(i, id0, id1, id2);
			if ( mesh->num_normals()  > 0 ) mesh->SetFaceNormalIds(i, id0, id1, id2);
			if ( mesh->num_uvs()      > 0 ) mesh->SetFaceUvIds    (i, id0, id1, id2);
			if ( mesh->num_colors()   > 0 ) mesh->SetFaceColorIds (i, id0, id1, id2);

		
		}

		mesh->mesh_type(Mesh::MT_TRIANGLES);
	}


	

	// Weight
	if (ai_mesh->HasBones())
	{
		mesh->UseBoneWeight();

		// We assume that maximum 4 bones can be related for a vertex at most.

		int *nums_bones_per_vertex = new int[ai_mesh->mNumVertices];

		// initialize
		for (unsigned int i = 0; i<ai_mesh->mNumVertices; i++)
		{
			nums_bones_per_vertex[i] = 0;
		}


		for (unsigned int i = 0; i<ai_mesh->mNumBones; i++)
		{
			for (unsigned int j = 0; j<ai_mesh->mBones[i]->mNumWeights; j++)
			{
				int vertex_id = ai_mesh->mBones[i]->mWeights[j].mVertexId;
				float weight = ai_mesh->mBones[i]->mWeights[j].mWeight;

				// If there are more than four joints related at one vertex,
				// we just skip the fifth ane later one. 
				if (nums_bones_per_vertex[vertex_id] == 4)
				{
					std::cerr << "static GL_RenderableObj* CreateRenderableObj(): Warning More than 4 bones are related to one vertex." << std::endl;
					continue;
				}


				int bone_id = GetBoneId(ai_mesh->mBones[i]->mName.C_Str());


				mesh->SetBoneIdAndWeight(vertex_id, nums_bones_per_vertex[vertex_id], bone_id, (double)weight);

				nums_bones_per_vertex[vertex_id]++;
			}


		}

		delete[] nums_bones_per_vertex;
	}


	// Material
	GL_Material *gl_material = nullptr;
	{
		if (ai_mesh->mMaterialIndex >= 0 && ai_mesh->mMaterialIndex < g_tmp_materials.size() )
		{
			gl_material = g_tmp_materials[ai_mesh->mMaterialIndex];
		}
		else
		{
			// aiMaterial *mat = ai_scene_->mMaterials[mesh->mMaterialIndex];
			// gl_material = CreateGlMaterial(mat);

			//???
			std::cerr << "static GL_RenderableObj* CreateRenderableObj(): No available material in g_tmp_materials" << std::endl;
		}
	}

	mesh->BuildVAO();
	GL_RenderableObj *r_obj = GL_ResourceManager::singleton()->CreateRenderableObj(obj_name, mesh, gl_material);
	if ( gl_material ) 
	{
		r_obj->flag_material(true);
	}


	// Set bone names and offset_matrices
	if ( ai_mesh->HasBones() )
	{
		r_obj->flag_skinning(true);
		for (unsigned int i = 0; i<ai_mesh->mNumBones; i++)
		{
			std::pair<std::string, cml::matrix44d> name_and_mat;

			name_and_mat.first = std::string(ai_mesh->mBones[i]->mName.C_Str());

			//aiNode *node = ai_scene->mRootNode->FindNode(ai_mesh->mBones[i]->mName);
		

			aiMatrix4x4 m = ai_mesh->mBones[i]->mOffsetMatrix;
			m.Inverse();


			for (int j = 0; j<4; j++)
				for (int k = 0; k<4; k++)
				{
					name_and_mat.second.set_basis_element(j, k
						, m[k][j]);
				}


			int bone_id = GetBoneId(name_and_mat.first);

			r_obj->SetBoneName(bone_id, name_and_mat.first);
			r_obj->SetBoneOffsetMatrix(bone_id, name_and_mat.second);
		}
	}

	return r_obj;
}


GL_RenderableObj* CreateRenderableRiggedObjByUsingAssimp(const std::string filename,
													const std::string renderable_obj_name,
													const std::map<std::string, int> &in_bonename_to_pmjoint,
												std::vector< std::pair<std::string, cml::matrix44d> > &out_bone_name_and_mat)
{

	g_ai_file_base_path = getBasePath(filename);
	mg::GL_VBOGroup* vbo_group = mg::GL_ResourceManager::singleton()->CreateVBOGroup();
	mg::GL_Material* gl_material = nullptr;
	bool flag_has_bone = false;

	//Assimp::Logger::LogSeverity severity = Assimp::Logger::NORMAL;
	Assimp::Logger::LogSeverity severity = Assimp::Logger::VERBOSE;

	// Create a logger instance for Console Output
	Assimp::DefaultLogger::create("", severity, aiDefaultLogStream_STDOUT);

	// Create a logger instance for File Output (found in project folder or near .exe)
	Assimp::DefaultLogger::create("assimp_log.txt", severity, aiDefaultLogStream_FILE);

	// Now I am ready for logging my stuff
	Assimp::DefaultLogger::get()->info("this is my info-call");


	Assimp::Importer ai_importer;

	const aiScene *ai_scene_ = ai_importer.ReadFile(filename, aiProcessPreset_TargetRealtime_Quality | aiProcess_Triangulate);


	// Kill it after the work is done
	Assimp::DefaultLogger::kill();

	//ai_scene_->mMaterials

	for (unsigned int i = 0; i<ai_scene_->mNumMeshes; i++)
	{
		const aiMesh *mesh = ai_scene_->mMeshes[i];
		if (mesh->HasBones())
		{
			vbo_group->Init(mesh->mNumVertices);

			// vertices
			vbo_group->SetVBO(GL_VBOGroup::VERTEX_VBO, GL_FLOAT, 3, (void*)mesh->mVertices);

			// normals
			vbo_group->SetVBO(GL_VBOGroup::NORMAL_VBO, GL_FLOAT, 3, (void*)mesh->mNormals);


			// tex_coords
			{
				if (mesh->GetNumUVChannels() >= 1)
				{
					if (mesh->mNumUVComponents[0] == 2)
					{
						// transform 
						for (unsigned int t_i = 0; t_i<mesh->mNumVertices; t_i++)
						{
							mesh->mTextureCoords[0][t_i][1] = 1 - mesh->mTextureCoords[0][t_i][1];
						}
						vbo_group->SetVBO(GL_VBOGroup::TEX_COORD_VBO, GL_FLOAT, 3, (void*)mesh->mTextureCoords[0]);
					}
				}
			}

			// color
			{
				if (mesh->GetNumColorChannels() >= 1)
				{
					vbo_group->SetVBO(GL_VBOGroup::COLOR_VBO, GL_FLOAT, 4, (void*)mesh->mColors[0]);
				}
			}



			// Triangle indices
			{
				unsigned int *tri_indices = new unsigned int[mesh->mNumFaces * 3];

				for (unsigned int i = 0; i<mesh->mNumFaces; i++)
				{
					if ( mesh->mFaces[i].mNumIndices == 1 ) 
					{
						tri_indices[i * 3 + 0] = mesh->mFaces[i].mIndices[0];
						tri_indices[i * 3 + 1] = mesh->mFaces[i].mIndices[0];
						tri_indices[i * 3 + 2] = mesh->mFaces[i].mIndices[0];
					}
					else if (mesh->mFaces[i].mNumIndices == 2)
					{
						tri_indices[i * 3 + 0] = mesh->mFaces[i].mIndices[0];
						tri_indices[i * 3 + 1] = mesh->mFaces[i].mIndices[1];
						tri_indices[i * 3 + 2] = mesh->mFaces[i].mIndices[1];
					}
					else
					{
						tri_indices[i * 3 + 0] = mesh->mFaces[i].mIndices[0];
						tri_indices[i * 3 + 1] = mesh->mFaces[i].mIndices[1];
						tri_indices[i * 3 + 2] = mesh->mFaces[i].mIndices[2];
					}
				}
				vbo_group->SetElementIndices(mesh->mNumFaces * 3, tri_indices);
				vbo_group->drawing_mode(GL_TRIANGLES);

				delete[] tri_indices;
			}

			// Return bone names and offset_matrices
			if (mesh->HasBones())
			{
				flag_has_bone = true;
				for (unsigned int i = 0; i<mesh->mNumBones; i++)
				{
					std::pair<std::string, cml::matrix44d> name_and_mat;

					name_and_mat.first = std::string(mesh->mBones[i]->mName.C_Str());

					aiNode *node = ai_scene_->mRootNode->FindNode(mesh->mBones[i]->mName);


					aiMatrix4x4 m = mesh->mBones[i]->mOffsetMatrix;
					m.Inverse();


					for (int j = 0; j<4; j++)
						for (int k = 0; k<4; k++)
						{
							name_and_mat.second.set_basis_element(j, k
								, m[k][j]);
						}



					out_bone_name_and_mat.push_back(name_and_mat);
				}
			}


			// Weight
			if (mesh->HasBones())
			{
				// We assume that maximum 4 bones can be related for a vertex at most.

				int *nums_bones_per_vertex = new int[mesh->mNumVertices];
				int *bone_ids = new int[4 * mesh->mNumVertices];
				double *bone_weights = new double[4 * mesh->mNumVertices];

				// initialize
				for (unsigned int i = 0; i<mesh->mNumVertices; i++)
				{
					nums_bones_per_vertex[i] = 0;
					bone_ids[i * 4 + 0] = bone_ids[i * 4 + 1] = bone_ids[i * 4 + 2] = bone_ids[i * 4 + 3] = -1;
					bone_weights[i * 4 + 0] = bone_weights[i * 4 + 1] = bone_weights[i * 4 + 2] = bone_weights[i * 4 + 3] = 0.0f;
				}


				for (unsigned int i = 0; i<mesh->mNumBones; i++)
				{
					for (unsigned int j = 0; j<mesh->mBones[i]->mNumWeights; j++)
					{
						int vertex_id = mesh->mBones[i]->mWeights[j].mVertexId;
						float weight = mesh->mBones[i]->mWeights[j].mWeight;

						// If there are more than four joints related at one vertex,
						// we just skip the fifth ane later one. 
						if (nums_bones_per_vertex[vertex_id] == 4)
						{
							continue;
						}


						int bone_id = i;
						if (in_bonename_to_pmjoint.find(mesh->mBones[i]->mName.C_Str()) != in_bonename_to_pmjoint.end())
						{
							bone_id = in_bonename_to_pmjoint.find(mesh->mBones[i]->mName.C_Str())->second;
						}
						bone_ids[vertex_id * 4 + nums_bones_per_vertex[vertex_id]] = bone_id;
						bone_weights[vertex_id * 4 + nums_bones_per_vertex[vertex_id]] = weight;

						nums_bones_per_vertex[vertex_id]++;
					}


				}

				vbo_group->SetVBO(GL_VBOGroup::BONE_ID_VBO, GL_INT, 4, bone_ids);
				vbo_group->SetVBO(GL_VBOGroup::BONE_WEIGHT_VBO, GL_DOUBLE, 4, bone_weights);


				delete[] nums_bones_per_vertex;
				delete[] bone_ids;
				delete[] bone_weights;
			}

			// Material
			{
				if (mesh->mMaterialIndex >= 0 && mesh->mMaterialIndex < ai_scene_->mNumMaterials)
				{
					aiMaterial *mat = ai_scene_->mMaterials[mesh->mMaterialIndex];

					//*out_material = CreateGlMaterial(mat);
					gl_material = CreateGlMaterial(mat);
				}
			}

			break;
		}
	}

	GL_RenderableObj *r_obj = GL_ResourceManager::singleton()->CreateRenderableObj(renderable_obj_name, vbo_group, gl_material);
	r_obj->flag_material(true);
	if ( flag_has_bone ) r_obj->flag_skinning(true);

	return r_obj;
}





//GL_RenderableObjGroup*
//GL_ResourceManager::ImportByAssimp(const std::string filename, std::string group_name)
//{
//	if ( group_name.empty() )
//	{
//		group_name = GenUniqueName("GROUP");
//	}
//
//	g_ai_file_base_path = getBasePath(filename);
//	g_tmp_materials.clear();
//
//
//	//Assimp::Logger::LogSeverity severity = Assimp::Logger::NORMAL;
//	Assimp::Logger::LogSeverity severity = Assimp::Logger::VERBOSE;
//
//	// Create a logger instance for Console Output
//	Assimp::DefaultLogger::create("", severity, aiDefaultLogStream_STDOUT);
//
//	// Create a logger instance for File Output (found in project folder or near .exe)
//	Assimp::DefaultLogger::create("assimp_log.txt", severity, aiDefaultLogStream_FILE);
//
//	// Now I am ready for logging my stuff
//	Assimp::DefaultLogger::get()->info("this is my info-call");
//
//	Assimp::Importer ai_importer;
//
//	const aiScene *ai_scene = ai_importer.ReadFile(filename, aiProcessPreset_TargetRealtime_Quality | aiProcess_Triangulate);
//
//
//	GL_RenderableObjGroup *renderable_group = GL_ResourceManager::singleton()->CreateRenderableObjGroup(group_name);
//
//	////////////////////////////////////
//	// Load Materials
//	for ( unsigned int i=0; i<ai_scene->mNumMaterials; i++ )
//	{
//		const aiMaterial *ai_mat = ai_scene->mMaterials[i];
//		std::string mat_name = GenUniqueName(group_name + "_MAT");
//		GL_Material *mat = CreateGlMaterial(ai_mat, mat_name);
//
//		g_tmp_materials.push_back(mat);
//	}
//
//
//	std::vector< const GL_RenderableObj *> gl_objs;
//	////////////////////////////////////
//	// Load Mesh
//	for ( unsigned int i=0; i<ai_scene->mNumMeshes; i++ )
//	{
//		const aiMesh *ai_mesh = ai_scene->mMeshes[i];
//		std::string obj_name = GenUniqueName(group_name+"_OBJ");
//		GL_RenderableObj *obj = mg::CreateRenderableObj(ai_mesh, obj_name);
//		
//		gl_objs.push_back(obj);
//		//ai_mesh->mPrimitiveTypes
//	}
//
//	////////////////////////////////////
//	// build the renderable_group. Hierarchically. 
//	std::stack<aiNode*> ainode_stack;
//	ainode_stack.push(ai_scene->mRootNode);
//	while ( !ainode_stack.empty() )
//	{
//		aiNode* node = ainode_stack.top();
//		ainode_stack.pop();
//
//		for ( unsigned int i=0; i<node->mNumChildren; i++ )
//		{
//			ainode_stack.push(node->mChildren[i]);
//		}
//
//		for ( unsigned int i=0; i<node->mNumMeshes; i++ )
//		{
//			const GL_RenderableObj *obj = gl_objs[node->mMeshes[i]];
//			
//
//			// Global transform
//			aiMatrix4x4 ai_t = node->mTransformation;
//			aiNode *parent_node = node->mParent;
//			while ( parent_node )
//			{
//				ai_t = parent_node->mTransformation * ai_t;
//				parent_node = parent_node->mParent;
//			}
//
//			// Convert
//			cml::matrix44d t;
//			for ( int j=0; j<4; j++ )
//				for ( int k=0; k<4; k++ )
//				{
//					t.set_basis_element(j, k, ai_t[k][j]);
//				}
//
//
//			renderable_group->AddRendereableObj(obj, t);
//		}
//	}
//
//	g_tmp_materials.clear();
//
//	// Kill it after the work is done
//	Assimp::DefaultLogger::kill();
//
//	return renderable_group;
//}


//GL_RenderableObjGroup*
//GL_ResourceManager::ImportByAssimp(const std::string filename, std::string group_name)
//{
//	if (group_name.empty())
//	{
//		group_name = GenUniqueName("GROUP");
//	}
//
//	g_ai_file_base_path = getBasePath(filename);
//	g_tmp_materials.clear();
//
//
//	//Assimp::Logger::LogSeverity severity = Assimp::Logger::NORMAL;
//	Assimp::Logger::LogSeverity severity = Assimp::Logger::VERBOSE;
//
//	// Create a logger instance for Console Output
//	Assimp::DefaultLogger::create("", severity, aiDefaultLogStream_STDOUT);
//
//	// Create a logger instance for File Output (found in project folder or near .exe)
//	Assimp::DefaultLogger::create("assimp_log.txt", severity, aiDefaultLogStream_FILE);
//
//	// Now I am ready for logging my stuff
//	Assimp::DefaultLogger::get()->info("this is my info-call");
//
//	Assimp::Importer ai_importer;
//
//	const aiScene *ai_scene = ai_importer.ReadFile(filename, aiProcessPreset_TargetRealtime_Quality | aiProcess_Triangulate);
//
//
//	GL_RenderableObjGroup *renderable_group = GL_ResourceManager::singleton()->CreateRenderableObjGroup(group_name);
//
//	////////////////////////////////////
//	// Load Materials
//	for (unsigned int i = 0; i<ai_scene->mNumMaterials; i++)
//	{
//		const aiMaterial *ai_mat = ai_scene->mMaterials[i];
//		std::string mat_name = GenUniqueName(group_name + "_MAT");
//		GL_Material *mat = CreateGlMaterial(ai_mat, mat_name);
//
//		g_tmp_materials.push_back(mat);
//	}
//
//
//	std::vector< const GL_RenderableObj *> gl_objs;
//	////////////////////////////////////
//	// Load Mesh
//	for (unsigned int i = 0; i<ai_scene->mNumMeshes; i++)
//	{
//		const aiMesh *ai_mesh = ai_scene->mMeshes[i];
//		std::string obj_name = GenUniqueName(group_name + "_OBJ");
//		GL_RenderableObj *obj = mg::CreateRenderableObj(ai_mesh, obj_name);
//
//		gl_objs.push_back(obj);
//		//ai_mesh->mPrimitiveTypes
//	}
//
//	////////////////////////////////////
//	// build the renderable_group. Hierarchically. 
//	std::stack<aiNode*> ainode_stack;
//	ainode_stack.push(ai_scene->mRootNode);
//	while (!ainode_stack.empty())
//	{
//		aiNode* node = ainode_stack.top();
//		ainode_stack.pop();
//
//		for (unsigned int i = 0; i<node->mNumChildren; i++)
//		{
//			ainode_stack.push(node->mChildren[i]);
//		}
//
//		for (unsigned int i = 0; i<node->mNumMeshes; i++)
//		{
//			const GL_RenderableObj *obj = gl_objs[node->mMeshes[i]];
//
//
//			// Global transform
//			aiMatrix4x4 ai_t = node->mTransformation;
//			aiNode *parent_node = node->mParent;
//			while (parent_node)
//			{
//				ai_t = parent_node->mTransformation * ai_t;
//				parent_node = parent_node->mParent;
//			}
//
//			// Convert
//			cml::matrix44d t;
//			for (int j = 0; j<4; j++)
//				for (int k = 0; k<4; k++)
//				{
//					t.set_basis_element(j, k, ai_t[k][j]);
//				}
//
//
//			renderable_group->AddRendereableObj(obj, t);
//		}
//	}
//
//	g_tmp_materials.clear();
//
//	// Kill it after the work is done
//	Assimp::DefaultLogger::kill();
//
//	return renderable_group;
//}

static cml::matrix44d aiMat2CmlMat(aiMatrix4x4 ai_t)
{
	cml::matrix44d t;

	for (int j = 0; j<4; j++)
		for (int k = 0; k<4; k++)
		{
			t(j, k) = ai_t[j][k];
		}

	return t;
}

void
GL_ResourceManager::ImportByAssimp(GL_SceneNode* out_root_node, const std::string filename, std::string name)
{
	if (name.empty())
	{
		name = GenUniqueName("SCENE");
	}
	out_root_node->name_ = name;

	g_ai_file_base_path = getBasePath(filename);
	g_tmp_materials.clear();
	g_bone_name_to_id.clear();
	g_count_bone = 0;

	//Assimp::Logger::LogSeverity severity = Assimp::Logger::NORMAL;
	Assimp::Logger::LogSeverity severity = Assimp::Logger::VERBOSE;

	// Create a logger instance for Console Output
	Assimp::DefaultLogger::create("", severity, aiDefaultLogStream_STDOUT);

	// Create a logger instance for File Output (found in project folder or near .exe)
	Assimp::DefaultLogger::create("assimp_log.txt", severity, aiDefaultLogStream_FILE);

	// Now I am ready for logging my stuff
	Assimp::DefaultLogger::get()->info("this is my info-call");

	Assimp::Importer ai_importer;

	const aiScene *ai_scene = ai_importer.ReadFile(filename, aiProcessPreset_TargetRealtime_Quality | aiProcess_Triangulate);



	////////////////////////////////////
	// Load Materials
	for (unsigned int i = 0; i<ai_scene->mNumMaterials; i++)
	{
		const aiMaterial *ai_mat = ai_scene->mMaterials[i];
		std::string mat_name = GenUniqueName(name + "_MAT");
		GL_Material *mat = CreateGlMaterial(ai_mat, mat_name);

		g_tmp_materials.push_back(mat);
	}


	std::vector< GL_RenderableObj *> gl_objs;
	////////////////////////////////////
	// Load Mesh
	for (unsigned int i = 0; i<ai_scene->mNumMeshes; i++)
	{
		const aiMesh *ai_mesh = ai_scene->mMeshes[i];

		std::string obj_name;
		{	
			if ( ai_mesh->mName.length > 0 )
			{
				obj_name = ai_mesh->mName.C_Str();
				obj_name += "_ROBJ";
			}
			else
			{
				obj_name = name + "_ROBJ";
			}
			obj_name = GenUniqueName(obj_name);
		}

		GL_RenderableObj *obj = mg::CreateRenderableObj(ai_mesh, obj_name);

		gl_objs.push_back(obj);
		//ai_mesh->mPrimitiveTypes
	}

	////////////////////////////////////
	// build the renderable_group. Hierarchically. 
	std::stack<GL_SceneNode*> gl_node_stack;
	gl_node_stack.push(out_root_node);

	std::stack<aiNode*> ainode_stack;
	ainode_stack.push(ai_scene->mRootNode);
	while (!ainode_stack.empty())
	{
		mg::GL_SceneNode* gl_node = gl_node_stack.top();
		gl_node_stack.pop();

		aiNode* node = ainode_stack.top();
		ainode_stack.pop();

		for (unsigned int i = 0; i<node->mNumChildren; i++)
		{
			ainode_stack.push(node->mChildren[i]);
			gl_node_stack.push(gl_node->CreateChild());
		}

		// node->name_
		gl_node->name_ = node->mName.C_Str();

		// node->mTransformation
		gl_node->transf().SetMat44( aiMat2CmlMat(node->mTransformation) );

		for (unsigned int i = 0; i<node->mNumMeshes; i++)
		{
			GL_RenderableObj *obj = gl_objs[node->mMeshes[i]];

			gl_node->AddRenderableObj(obj);
		}
	}

	g_tmp_materials.clear();

	// Kill it after the work is done
	Assimp::DefaultLogger::kill();

}

};
