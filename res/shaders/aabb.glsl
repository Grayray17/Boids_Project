#version 330 core

uniform mat4 uProjectionMatrix;
uniform mat4 uModelViewMatrix;

uniform vec3 uColor;

uniform vec3 uMax;
uniform vec3 uMin;

#ifdef _VERTEX_

flat out int v_instanceID;

void main() {
	v_instanceID = gl_InstanceID;
}

#endif

#ifdef _GEOMETRY_

layout(points) in;
layout(line_strip, max_vertices = 2) out;

flat in int v_instanceID[];

const int indices[] = int[](
	0, 1, 0, 2, 0, 4,
	1, 3, 1, 5, 2, 3,
	2, 6, 3, 7, 4, 5,
	4, 6, 5, 7, 6, 7
);

void main() {

	vec4 points[] = vec4[](
		vec4(uMax.x, uMax.y, uMax.z, 1),
		vec4(uMax.x, uMax.y, uMin.z, 1),
		vec4(uMax.x, uMin.y, uMax.z, 1),
		vec4(uMax.x, uMin.y, uMin.z, 1),
		vec4(uMin.x, uMax.y, uMax.z, 1),
		vec4(uMin.x, uMax.y, uMin.z, 1),
		vec4(uMin.x, uMin.y, uMax.z, 1),
		vec4(uMin.x, uMin.y, uMin.z, 1)
	);

	gl_Position = uProjectionMatrix * uModelViewMatrix * points[indices[v_instanceID[0]*2]];
	EmitVertex();

	gl_Position = uProjectionMatrix * uModelViewMatrix * points[indices[v_instanceID[0]*2+1]];
	EmitVertex();
	EndPrimitive();
}

#endif

#ifdef _FRAGMENT_

out vec3 f_color;

void main() {
	f_color = uColor;
}

#endif