#version 330

// data from vertex shader
in vec3 outNormal;
in vec3 outToLight;
in vec3 outToCamera;

// color for framebuffer
out vec4 resultingColor;

/////////////////////////////////////////////////////////

// parameters of the light and possible values
uniform vec3 ambientColor; // = vec3(0.6, 0.3, 0);
uniform vec3 diffuseColor; // = vec3(1, 0.5, 0);
uniform vec3 specularColor; // = vec3(0, 1, 0);
uniform float shininess; // = 64;


/////////////////////////////////////////////////////////

// returns intensity of diffuse reflection
vec3 diffuseLighting(in vec3 N, in vec3 L)
{
   // calculation as for Lambertian reflection
   return diffuseColor * clamp(dot(N, L), 0, 1);
}

// returns intensity of specular reflection
vec3 specularLighting(in vec3 N, in vec3 L, in vec3 V)
{
   float specularTerm = 0;

   // calculate specular reflection only if
   // the surface is oriented to the light source
   if(dot(N, L) > 0)
   {
      // half vector
      vec3 H = normalize(L + V);
      specularTerm = pow(dot(N, H), shininess);
   }
   return specularColor * specularTerm;
}

void main(void)
{
   // normalize vectors after interpolation
   vec3 L = normalize(outToLight);
   vec3 V = normalize(outToCamera);
   vec3 N = normalize(outNormal);

   // combination of all components and diffuse color of the object
   resultingColor.xyz = ambientColor + diffuseLighting(N, L) + specularLighting(N, L, V);
   resultingColor.a = 1;
} 
