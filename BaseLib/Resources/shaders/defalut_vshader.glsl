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
	
uniform bool flag_shadowmap = false;
uniform mat4 shadowmap_tex_bias_light_pov_matrix;
out vec4 fs_shadowmap_coord;

out vec4 fs_color;
out vec2 fs_uv;
out vec3 fs_modelviewed_normal;
out vec3 fs_modelviewed_pos;
	
void main(void)
{
	fs_color = vs_color;
	fs_uv = vs_uv;
		
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
	fs_modelviewed_normal = normalize(modelviewbone_matrix*vec4(vs_normal, 0.0)).xyz;
	
	fs_modelviewed_pos = (modelviewbone_matrix * vs_position).xyz;

	if ( flag_shadowmap )
	{
		fs_shadowmap_coord = shadowmap_tex_bias_light_pov_matrix * modelbone_matrix * vs_position;
	}
}