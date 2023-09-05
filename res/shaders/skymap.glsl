#version 330 core

uniform mat4 uProjectionMatrix;
uniform mat4 uModelViewMatrix;

uniform float uZDistance;

uniform sampler2D uSkyMap;

const float PI = 3.14159265359f;

#ifdef _VERTEX_

flat out int v_instanceID;

void main() {
	v_instanceID = gl_InstanceID;
}

#endif

#ifdef _GEOMETRY_

layout(points) in;
layout(triangle_strip, max_vertices = 3) out;

flat in int v_instanceID[];

out vec3 v_world_space;

const vec3 points[] = vec3[](
	vec3(-1, -1, -1),
	vec3(1, -1, -1),
	vec3(-1, 1, -1),
	vec3(1, 1, -1),
	vec3(-1, -1, 1),
	vec3(1, -1, 1),
	vec3(-1, 1, 1),
	vec3(1, 1, 1)
);

const int indices[] = int[](
	0, 1, 3,
	0, 3, 2,
	0, 5, 1,
	0, 4, 5,
	0, 6, 4,
	0, 2, 6,
	1, 5, 7,
	3, 1, 7,
	2, 3, 7,
	6, 2, 7,
	4, 6, 7,
	5, 4, 7
);

void main() {

	v_world_space = uZDistance * points[indices[v_instanceID[0]*3]];
	gl_Position = uProjectionMatrix * vec4(mat3(uModelViewMatrix) * v_world_space, 1);
	EmitVertex();

	v_world_space = uZDistance * points[indices[v_instanceID[0]*3+1]];
	gl_Position = uProjectionMatrix * vec4(mat3(uModelViewMatrix) * v_world_space, 1);
	EmitVertex();

	v_world_space = uZDistance * points[indices[v_instanceID[0]*3+2]];
	gl_Position = uProjectionMatrix * vec4(mat3(uModelViewMatrix) * v_world_space, 1);
	EmitVertex();
	EndPrimitive();
}

#endif

#ifdef _FRAGMENT_

in vec3 v_world_space;

out vec3 f_color;

vec2 latlong(vec3 v) {
    v = normalize(v);
    //v.xz = -v.xz;
    float theta = acos(-v.y);
    float phi = atan(v.z, v.x) + PI / 2;
    return vec2(0.5 + phi / (2.0 * PI), theta / PI);
}

void main() {
	f_color = texture(uSkyMap, latlong(v_world_space)).rgb;
	// f_color = vec3(0,0,0);
}

#endif