#version 130

uniform sampler2D depth_texture;
uniform vec3 cam_pos;
uniform float zNear;
uniform float zFar;
uniform float textureLength;
varying vec2 texCoord;


vec3 unpackColor(float f) {
    vec3 color;
    color.b = floor(f / 256.0 / 256.0);
    color.g = floor((f - color.b * 256.0 * 256.0) / 256.0);
    color.r = floor(f - color.b * 256.0 * 256.0 - color.g * 256.0);
    // now we have a vec3 with the 3 components in range [0..256]. Let's normalize it!
    return color / 256.0;
}	

void main (void)  
{
	vec3 texcolor =  vec3(texture2D(depth_texture, texCoord));
	
	float zn = zNear;
	float zf = zFar;
	float texture_length = textureLength;
	
	float val = texcolor.x;
	float z_b = texture2D(depth_texture, texCoord).x;
    float z_n = 2.0 * z_b - 1.0;
    float z_e = 2.0 * zn * zf / (zf + zn - z_n * (zf - zn));
	
	vec2 coord = gl_FragCoord.xy;
	float distance_coord = 2*(sqrt((coord.x - (texture_length/2))*(coord.x - (texture_length/2)) + (coord.y - (texture_length/2))*(coord.y - (texture_length/2)))/texture_length) ;
	
	float cos_thetha = 1.0 / (sqrt(1.0 + distance_coord*distance_coord));
	 
	float value = (z_e-zn)/(zf-zn);
	
	float val2 = z_e/cos_thetha;

	val2 /= zf;
	
	//To make the value between 0 to 1 => 0 16777216 so that the precision is maintained
	val2 *= 16777216;
	
	vec3 distance_value = unpackColor(val2);
		
	gl_FragColor = vec4(distance_value, 1.0);
}  