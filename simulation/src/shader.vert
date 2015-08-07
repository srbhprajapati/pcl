#version 120

attribute vec3 position;
attribute vec2 texelvalue;
varying vec2 texCoord;


void main(void)
{
   gl_Position = gl_ModelViewProjectionMatrix*vec4(position,1.0);
   texCoord = texelvalue;
}  