
// handle to texture
uniform sampler2D image;

uniform float kernel[25];

uniform float width;
uniform float height;
    						
vec4 gauss(){
	vec4 vColor = vec4(0.0, 0.0, 0.0, 0.0);
	    						
	int i = 0;
	for(int y=-2; y<=2; y++){
		for(int x=-2; x<=2; x++){
			vColor += texture2D(image, gl_TexCoord[0].st + vec2(x/width,y/height)) * kernel[i];
			i++;
		}
	}
	
	vColor = vColor / 115.0;
	
	return vColor;
}

void main(){
	gl_FragColor = gauss();
}

