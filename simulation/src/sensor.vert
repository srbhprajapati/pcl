#version 120

attribute vec3 position;
attribute vec3 color;
varying vec3 color2;

void main(void)
{
   gl_Position = gl_ModelViewProjectionMatrix*vec4(position,1.0);
   color2 = color;
}  