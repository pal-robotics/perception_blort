varying vec4 vertex_model;

uniform sampler2D tex_frame_color;
uniform sampler2D tex_model_color;

uniform mat3 mOffsetX;
uniform mat3 mOffsetY;
uniform mat4 modelviewprojection; // -matrix of previouse tracked position

// TODO remove threshold
uniform float fTol; // tolerance for angle comparison
uniform bool compare;
uniform bool analyze;
uniform bool textured;
uniform int kernelsize;

uniform vec4 drawcolor;

const float pi = 3.141592654;
const float dpi = 0.318309886;  // dpi = 1 / pi;

// TODO remove threshold
const float fTol_color = 0.2;
const float fTol_h = 10.0;
const float fTol_s = 0.2;
const float fTol_v = 0.2;

// predefined colors
const vec4 white	= vec4(1.0, 1.0, 1.0, 1.0);
const vec4 black 	= vec4(0.0, 0.0, 0.0, 0.0);
const vec4 red 		= vec4(1.0, 0.0, 0.0, 0.0);
const vec4 green 	= vec4(0.0, 1.0, 0.0, 0.0);
const vec4 blue 	= vec4(0.0, 0.0, 1.0, 0.0);
const vec4 yellow = vec4(1.0, 1.0, 0.0, 0.0);
const vec4 orange = vec4(1.0, 0.5, 0.0, 0.0);

vec3 rgb2hsv(vec3 rgb){
	vec3 hsv;
	float h,s,v;
	
	float fMax = rgb.r;
	float fMin = rgb.r;
	if(rgb.g > fMax) fMax = rgb.g;
	if(rgb.b > fMax) fMax = rgb.b;
	if(rgb.g < fMin) fMin = rgb.g;
	if(rgb.b < fMin) fMin = rgb.b;
	
	if(fMax-fMin < 0.01) h = 0.0;
	else if(fMax == rgb.r) h = 60.0 * (0.0 + (rgb.g-rgb.b)/(fMax-fMin));
	else if(fMax == rgb.g) h = 60.0 * (2.0 + (rgb.b-rgb.r)/(fMax-fMin));
	else if(fMax == rgb.b) h = 60.0 * (4.0 + (rgb.r-rgb.g)/(fMax-fMin));
	
	if(h < 0.0) h = h + 360.0;
	
	if(fMax < 0.01) s = 0.0;
	else s = (fMax-fMin)/fMax;
	
	v = fMax;
		
	hsv = vec3(h, s, v);

	return hsv;
}

bool compareColor(vec3 color_model, vec3 color_frame){
	vec3 hsv_model = rgb2hsv(color_model);
	vec3 hsv_frame = rgb2hsv(color_frame);
	bool h,s,v;
	
	return ( h=abs(hsv_model.x-hsv_frame.x)<fTol_h );
// 	return (length(color_frame-color_model)<fTol_color);
}

bool compareNeighbourPixels(vec4 color_model, vec4 texcoords_frame){
	vec4 color_frame;
	
	for(int r=1; r<=kernelsize; r++){
		for(int i=0; i<3; i++){
			for(int j=0; j<3; j++){
				if(i!=1 && j!=1){
					color_frame = texture2D(tex_frame_color, texcoords_frame.xy + vec2(mOffsetX[i][j]*float(r), mOffsetY[i][j]*float(r)));
					return compareColor(color_model, color_frame);
				}
			}
		}
	}
	
	return false;
}

void main(){
	vec4 texcoords_model;
	vec4 texcoords_frame;
	vec4 color_frame;
	vec4 color_model;
	
	// draw all model edges
	if(!compare) return;
	
	// get color of model
	texcoords_model = modelviewprojection * vertex_model;
	texcoords_model.x = (texcoords_model.x / texcoords_model.w + 1.0) * 0.5;
	texcoords_model.y = (texcoords_model.y / texcoords_model.w + 1.0) * 0.5;
	color_model = texture2D(tex_model_color, texcoords_model.xy);
		
	// get color of frame
	texcoords_frame = gl_ModelViewProjectionMatrix * vertex_model;
	texcoords_frame.x = (texcoords_frame.x / texcoords_frame.w + 1.0) * 0.5;
	texcoords_frame.y = (texcoords_frame.y / texcoords_frame.w + 1.0) * 0.5;	
	color_frame = texture2D(tex_frame_color, texcoords_frame.xy);
	
	if(compareColor(color_model, color_frame)){
		gl_FragColor = red;
	}else{
		if(compareNeighbourPixels(color_model, texcoords_frame)){
			gl_FragColor = orange;
		}else{
			gl_FragColor = blue;
			if(!analyze)
				discard;
		}
	}

}



