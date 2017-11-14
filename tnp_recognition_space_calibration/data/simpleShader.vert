#version 330 core
layout(location = 0) in vec3 vertexPosition;
uniform mat4 camera;
uniform vec3 ambientColor;

out vec4 vert_color;

void main()
{
   gl_Position = camera * vec4(vertexPosition,1);
   vert_color = vec4(ambientColor,1);
}
