#version 330
				
layout (location=0) in vec4 vs_position;
layout (location=1) in vec4 vs_color;
layout (location=2) in vec3 vs_normal;
layout (location=3) in vec2 vs_uv;
layout (location=4) in uvec4 vs_bone_id;
layout (location=5) in vec4 vs_bone_weight;
	
uniform mat4 projection_matrix;
uniform mat4 view_matrix;
uniform mat4 model_matrix;
	
uniform bool flag_skinning;
uniform mat4 bone_matrices[100];
	
	
void main(void)
{
	// mat4 modelview_matrix = view_matrix * model_matrix;
	mat4 modelbone_matrix;
	// mat4 modelviewbone_matrix;
	if ( !flag_skinning ) 
	{	
		//modelviewbone_matrix = modelview_matrix;
		modelbone_matrix = model_matrix;
	}
	else
	{	
		//modelviewbone_matrix = modelview_matrix *
		modelbone_matrix = model_matrix * 
					(
						vs_bone_weight[0] * bone_matrices[vs_bone_id[0]] 
					+ vs_bone_weight[1] * bone_matrices[vs_bone_id[1]] 
					+ vs_bone_weight[2] * bone_matrices[vs_bone_id[2]]
					+ vs_bone_weight[3] * bone_matrices[vs_bone_id[3]] 
					);
	}
	
	mat4 modelviewbone_matrix =  view_matrix * modelbone_matrix;
	gl_Position = projection_matrix * (modelviewbone_matrix * vs_position);

}