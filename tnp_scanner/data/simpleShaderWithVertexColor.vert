#version 330 core
layout(location = 0) in vec3 vertexPosition;
layout(location = 1) in vec3 vertexColor;
uniform mat4 camera;

out vec4 vert_color;

void main()
{
   gl_Position = camera * vec4(vertexPosition,1);
   vert_color = vec4(vertexColor,1);
}
