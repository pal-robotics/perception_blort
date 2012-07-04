varying vec4 vertex_model;

uniform sampler2D tex_model_edge;
uniform sampler2D tex_frame_edge;

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

// predefined colors
const vec4 white	= vec4(1.0, 1.0, 1.0, 1.0);
const vec4 black 	= vec4(0.0, 0.0, 0.0, 0.0);
const vec4 red 		= vec4(1.0, 0.0, 0.0, 0.0);
const vec4 green 	= vec4(0.0, 1.0, 0.0, 0.0);
const vec4 blue 	= vec4(0.0, 0.0, 1.0, 0.0);
const vec4 yellow = vec4(1.0, 1.0, 0.0, 0.0);
const vec4 orange = vec4(1.0, 0.5, 0.0, 0.0);


bool compareNeighbourPixels(vec4 edge_model, vec4 texcoords_frame){
	vec4 edge_frame;
	
	for(int r=1; r<=kernelsize; r++){
		for(int i=0; i<3; i++){
			for(int j=0; j<3; j++){
				if(i!=1 && j!=1){
					edge_frame = texture2D(tex_frame_edge, texcoords_frame.xy + vec2(mOffsetX[i][j]*r, mOffsetY[i][j]*r));
					if(length(edge_frame-edge_model)<fTol_color){
						return true;
					}
				}
			}
		}
	}
	
	return false;
}

void main(){
	vec4 texcoords_model;
	vec4 texcoords_frame;
	vec4 edge_model;
	vec4 edge_frame;
	vec2 gradient_frame;
	vec2 gradient_model;
	float alpha;
	
	// get edge information of model
	texcoords_model = modelviewprojection * vertex_model;
	texcoords_model.x = (texcoords_model.x / texcoords_model.w + 1.0) * 0.5;
	texcoords_model.y = (texcoords_model.y / texcoords_model.w + 1.0) * 0.5;
	edge_model = texture2D(tex_model_edge, texcoords_model.xy);
	
	// pixel of model is not an edge pixel
	if( edge_model.z<0.01 ){
		discard;
		return;
	}
	
	if( !compare ){
		gl_FragColor = drawcolor;
		return;
	}	
	
	// get edge information of frame
	texcoords_frame = gl_ModelViewProjectionMatrix * vertex_model;
	texcoords_frame.x = (texcoords_frame.x / texcoords_frame.w + 1.0) * 0.5;
	texcoords_frame.y = (texcoords_frame.y / texcoords_frame.w + 1.0) * 0.5;	
	edge_frame = texture2D(tex_frame_edge, texcoords_frame.xy);
	
	// pixel of frame is not an edge pixel
	if( edge_frame.z<0.01 ){
		//discard;
		gl_FragColor = black;
		if(!analyze)
			discard;
		return;
	}
	
// 	if( !compare ){
// 		gl_FragColor = drawcolor;
// 		return;
// 	}	
	
	if(textured){
		// compare gradients and magnitude (gx, gy, m)
		if(length(edge_frame - edge_model) < fTol_color){
			gl_FragColor = green;
		}else{
			if(compareNeighbourPixels(edge_model, texcoords_frame)){
				gl_FragColor = yellow;
			}else{
				gl_FragColor = red;
				if(!analyze)
					discard;
				return;
			}
		}
	}else{
	
		// calculate edge gradients of frame and model
		gradient_frame = (edge_frame.xy - 0.5) * 2.0;
		gradient_model = (edge_model.xy - 0.5) * 2.0;
		
		gradient_frame = normalize(gradient_frame);
		gradient_model = normalize(gradient_model);
		
		// calculate edge angle between frame and model; range(alpha) = [0...180]
		alpha = acos(gradient_frame.x * gradient_model.x + gradient_frame.y * gradient_model.y);
		
		// compare edges
		if( (alpha < (0.0 + fTol)) || (alpha > (pi - fTol)) ){
			gl_FragColor = green;
		}else{
			gl_FragColor = red;
			if(!analyze)
				discard;
			return;
		}
	}
}



