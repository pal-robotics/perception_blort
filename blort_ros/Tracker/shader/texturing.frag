varying vec4 vertex;

uniform sampler2D texture;

uniform mat4 modelviewprojection;

uniform bool useTexCoords=false;

uniform float x;
uniform float y;
uniform float w;
uniform float h;

void main(){
	vec4 texcoords;
	float z;
	
// 	gl_FragColor = vec4(1.0,0.0,0.0,0.0);
	
	if(useTexCoords){
		z = 1.0 / gl_FragCoord.z;
		texcoords.x = gl_TexCoord[0].x * z;
		texcoords.y = gl_TexCoord[0].y * z;
		gl_FragColor = texture2D(texture, gl_TexCoord[0].xy);
	}else{
		// calculate texture coordinats
		texcoords = modelviewprojection * vertex;
		texcoords.x = (texcoords.x / texcoords.w + 1.0) * 0.5;
		texcoords.y = (texcoords.y / texcoords.w + 1.0) * 0.5;
		
		// Crop texcoords from window to texture coordinates
		texcoords.x = (texcoords.x-x)/w;
		texcoords.y = (texcoords.y-y)/h;
		
		// get texture color for frame and model
		gl_FragColor =  texture2D(texture, texcoords.xy);
	}
}



