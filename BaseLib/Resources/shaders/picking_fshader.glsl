#version 330
				

layout (location = 0) out vec4 color;

// Values that stay constant for the whole mesh.
uniform vec3 picking_name = vec3(0, 0, 0);

void main()
{
	color.xyz = picking_name;
}