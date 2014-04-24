#version 120

uniform vec3 cam_pos;

varying vec3 color2;

float PlaneDistance(in vec3 point, in vec3 normal, in float pDistance)
{
	return dot(point - (normal * pDistance), normal);

}

float SphereDistance(in vec3 point, in float radius)
{
	return (length(point) - radius);
}

float opRep(vec3 p, vec3 c)
{
	vec3 q = mod(p,c) - 0.5*c;
	return SphereDistance(q, 2.0);
}


bool raymarch(vec3 ray_start, vec3 ray_dir, out float dist, out vec3 p)
{
	dist = 0.0;
	float minStep = 0.01;
	p = ray_start + ray_dir * dist;
	
	for(int i=0; i<400; i++)
	{
		float radius = 4.0;
		float mapDist = opRep(p, vec3(5.0, 5.0, 5.0));
		if (mapDist<0.000001)
		{
			return true;
		}
		
		p += ray_dir*mapDist;
		dist += mapDist;
	}
	return false;
	
}

void main (void)  
{   
	vec2 resolution = vec2(640, 480);
	vec2 position = vec2((gl_FragCoord.x - (resolution.x / 2.0) - 640) / resolution.y, (gl_FragCoord.y - (resolution.y / 2.0)) / resolution.y);
	position *= 20.0;
	vec3 ray_start = vec3(0,0,15.0);
	vec3 ray_dir = normalize(vec3(position,5.0) - ray_start);
	
	vec3 p;
	float dist;
	if(raymarch(ray_start, ray_dir, dist, p))
	{
		dist /= 30.0;
		gl_FragColor = vec4(1.0-dist, 1.0-dist, 1.0-dist, 1.0);
	}
	else gl_FragColor = vec4(0.0, 0.0, 0.0, 1.0);  
}  