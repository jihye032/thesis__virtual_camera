#version 330
				
in vec4 fs_color;
in vec3 fs_normal;
in vec2 fs_uv;
in vec3 fs_modelviewed_pos;
in vec3 fs_modelviewed_normal;
in vec4 fs_shadowmap_coord;

layout (location = 0) out vec4 color;

struct Light
{
	// 0: Directionl Light
	// 1: Point Light
	// 2: Spot Light
	int type;


	vec3 dir;
	vec3 position;
	vec4 intensity; // I_l
	float cos_cutoff;
};

uniform int num_lights = 20;
uniform Light lights[20];
uniform bool flag_lights[20];

uniform float I_a = 1.0f;				  // ambient light intensity
uniform vec4 K_a =  vec4(0.5f, 0.5f, 0.5f, 1.f);
uniform vec4 K_d =  vec4(0.5f, 0.5f, 0.5f, 1.f);
uniform vec4 K_s =  vec4(0.1f, 0.1f, 0.1f, 1.f);

uniform float shininess_n = 100.f;	
uniform int shading_mode = 0;	// 1: Phong Shading, 2: Flat Shading, 0: No Shading

uniform bool flag_use_vertex_color = true;
uniform bool flag_texture = false;
uniform sampler2D tex0;

uniform bool flag_highlight = false;
	

uniform bool flag_shadowmap = false;
uniform sampler2D tex7_shadowmap;
vec2 poisson_disk[4] = vec2[](
		vec2( -0.94201624, -0.39906216 ),
		vec2( 0.94558609, -0.76890725 ),
		vec2( -0.094184101, -0.92938870 ),
		vec2( 0.34495938, 0.29387760 )
	);

void main()
{
		

	vec3 K_a_, K_d_, K_s_;
	float a_ = (K_a.a+K_d.a+K_s.a)/3.f;


	if ( flag_texture && flag_use_vertex_color )
	{
		K_d_ = min(texture(tex0, fs_uv).rgb * fs_color.rgb, 1.0);
		K_a_ = K_d_*0.2;
		a_ = texture(tex0, fs_uv).a * fs_color.a;
	}
	else if ( flag_texture )
	{
		K_d_ = min(texture(tex0, fs_uv).rgb, 1.0);
		K_a_ = K_d_*0.2;
		a_ = texture(tex0, fs_uv).a;
	}
	else if ( flag_use_vertex_color )
	{
		K_d_ = fs_color.rgb;
		K_a_ = K_d_*0.2;
		a_ = fs_color.a;
	}
	else
	{
		K_a_ = K_a.rgb;
		K_d_ = K_d.rgb;
	}
	K_s_ = K_s.rgb;
		
		
		
	if ( shading_mode == 0 )
	{
		color = vec4(K_d_, a_);
	}
	else if ( shading_mode == 1 || shading_mode == 2 )
	{
		vec3 L, H, V, N;

		vec3 ambient = I_a * K_a_;
		vec3 diffuse = vec3(0.f);
		vec3 specular = vec3(0.f);


		// View Direction
		V = normalize(-fs_modelviewed_pos);

		// Normal Direction
		if ( shading_mode == 1 ) 
		{
			N = normalize(fs_modelviewed_normal);
		}
		else
		{
			vec3 fdx = vec3(dFdx(fs_modelviewed_pos));
			vec3 fdy = vec3(dFdy(fs_modelviewed_pos)); 
			N = normalize(cross(fdx,fdy));
		}

		bool no_light = true;
		for ( int i=0; i<num_lights; i++ )
		{
			if ( !flag_lights[i] ) continue;
			no_light = false;

			// Directional Light
			if ( lights[i].type == 0 )
			{
				L = normalize(-lights[i].dir);
				H = (L+V)/length(L+V);

				vec4 I_l = lights[i].intensity;

				diffuse  += I_l.a * I_l.rgb * K_d_ * max(0.f, dot(L, N));
				specular += I_l.a * I_l.rgb * K_s_ * pow(max(0.f, dot(N, H)), shininess_n);
			}

			// Point Light
			else if ( lights[i].type == 1 )
			{
				L = normalize(lights[i].position-fs_modelviewed_pos);
				H = (L+V)/length(L+V);
				float d =  length(lights[i].position-fs_modelviewed_pos);

				vec4 I_l = lights[i].intensity;
				if ( d > 0.001f )
					I_l = lights[i].intensity * min(1.0f/(0.001*d*d), 1.0f);	

				diffuse  += I_l.a * I_l.rgb * K_d_ * max(0.f, dot(L, N));
				specular += I_l.a * I_l.rgb * K_s_ * pow(max(0.f, dot(N, H)), shininess_n);
			}

			// Spot Light
			else if ( lights[i].type == 2 )
			{
				L = normalize(lights[i].position-fs_modelviewed_pos);
				H = (L+V)/length(L+V);
				vec3 Sd = normalize(-lights[i].dir);

				vec4 I_l = lights[i].intensity;

				if ( dot(Sd,L) >= lights[i].cos_cutoff )
				{
					diffuse  += I_l.a * I_l.rgb * K_d_ * max(0.f, dot(L, N));
					specular += I_l.a * I_l.rgb * K_s_ * pow(max(0.f, dot(N, H)), shininess_n);
				}
			}

				
		}

		if ( no_light )
		{
			color.rgb = K_d_.rgb;
			color.a = a_;
		}
		else
		{
			color.rgb = min(ambient + diffuse + specular, 1.0);
			color.a = a_;
		}

		if ( flag_shadowmap )
		{
			vec4 shadowmap_coord_divided = fs_shadowmap_coord / fs_shadowmap_coord.w ;
			// Used to lower moire pattern and self-shadowing
			shadowmap_coord_divided.z -= 0.0001;

			float visibility = 1.0;
			for (int i=0;i<4;i++)
			{
				float neartest_distance_from_light = texture2D(tex7_shadowmap, shadowmap_coord_divided.st + poisson_disk[i]/700.0).z;
 				if (fs_shadowmap_coord.w > 0.0 &&  neartest_distance_from_light < shadowmap_coord_divided.z )
 					visibility -= 0.15;
			}
			color = visibility*color;
		}

		if ( flag_highlight )
		{
			vec3 h_color = vec3(1., 1., 0);
			V = normalize(-fs_modelviewed_pos);
			float d = dot(fs_modelviewed_normal, V);
			color.rgb = min( d*color.rgb + 2.0*(1.0-d)*h_color , 1.0);

			// contour
			if ( d < 0.5 )
				color = vec4(0, 0, 0, 0);
		}
	}
	
}