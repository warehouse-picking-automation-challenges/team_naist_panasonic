#version 330

// attributes
layout(location = 0) in vec3 vertexPosition; // xyz - position
layout(location = 1) in vec3 vertexNormal; // xyz - normal

// matrices
uniform mat4 modelMat;
uniform mat4 viewMat;
uniform mat4 projMat;
uniform mat3 normalMat;

// position of light and camera
uniform vec3 lightPos;
uniform vec3 cameraPos;

// data for fragment shader
out vec3 outNormal;
out vec3 outToLight;
out vec3 outToCamera;

///////////////////////////////////////////////////////////////////

void main(void)
{
   // position in world space
   vec4 worldPosition = modelMat * vec4(vertexPosition, 1);

   // normal in world space
   outNormal = normalize(normalMat * vertexNormal);

   // direction to light
   outToLight = normalize(lightPos - worldPosition.xyz);

   // direction to camera
   outToCamera = normalize(cameraPos - worldPosition.xyz);

   // screen space coordinates of the vertex
   gl_Position = projMat * viewMat * worldPosition;
} 
