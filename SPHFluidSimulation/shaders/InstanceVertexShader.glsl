#version 400

uniform mat4 view;
uniform mat4 projection;

in vec3 vertexPosition;
in vec3 vertexNormal;
in vec2 texCoord;
in mat4 model;

out vec3 worldNormal;
out vec2 uv;

void main()
{
	mat4 clip = projection * view * model;
	worldNormal = mat3(clip) * vertexNormal;
	uv = vec2(texCoord);
	gl_Position = clip * vec4(vertexPosition, 1.0);
}