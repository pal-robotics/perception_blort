
uniform sampler2D texFrame;     // image with result of preprocessing

varying vec4 vertex_model;
varying vec2 vEdgeDirection;           // direction of edge of geometry object

uniform float width;    // fWidthDivided = 1 / width_of_image_in_pixel;
uniform float height;   // fHeightDivided = 1 / height_of_image_in_pixel;
const float fTol = 0.3;             // tolerance for deviation of angle of edge

uniform bool analyze;

uniform vec4 drawcolor;

const float pi = 3.141592654;   
const float dpi = 0.318309886;  // dpi = 1 / pi;
const float inf = 100000.0;

const vec4 white	= vec4(1.0, 1.0, 1.0, 1.0);
const vec4 red 		= vec4(1.0, 0.0, 0.0, 0.0);
const vec4 green 	= vec4(0.0, 1.0, 0.0, 0.0);
const vec4 blue 	= vec4(0.0, 0.0, 1.0, 0.0);
const vec4 black 	= vec4(0.0, 0.0, 0.0, 0.0);

void main(){
	vec4 texcoords_frame;
	vec2 vEdgeNormal;
    
	// get color of corresponding pixel of texture of camera image
	texcoords_frame = gl_ModelViewProjectionMatrix * vertex_model;
	texcoords_frame.x = (texcoords_frame.x / texcoords_frame.w + 1.0) * 0.5;
	texcoords_frame.y = (texcoords_frame.y / texcoords_frame.w + 1.0) * 0.5;
	
	//vec2 vTexCoord = vec2(gl_FragCoord.x / width, gl_FragCoord.y / height);
	vec4 vColor = texture2D(texFrame, vec2(texcoords_frame.x, texcoords_frame.y) );
	vEdgeNormal = vColor.xy;
	vEdgeNormal = (vEdgeNormal - 0.5) * 2.0;  // scale to range [-1 ... 1]
	vEdgeNormal = normalize(vEdgeNormal);

	// if pixel is not detected as edge pixel during preprocessing, drop pixel (gl_FragDepth = 1000.0)
	if(vColor.z < 0.01){
		gl_FragColor = black;
		if(!analyze)
			discard;
		return;
	}
		
	float len = length(vEdgeDirection.xy-vEdgeNormal.xy);
	if(len < fTol || len > (2.0-fTol)){
		gl_FragColor = green;
		return;
	}else{
		gl_FragColor = red;
		if(!analyze)
			discard;
		return;
	}
	

// 	float alpha = acos(vEdgeDirection.x * vEdgeNormal.x + vEdgeDirection.y * vEdgeNormal.y);
// 	
// 	// if edges are not aligned, drop pixel (gl_FragDepth = 1000.0)
// 	if( (alpha < fTol) && (alpha > (pi - fTol)) ){
// 		// this pixel alignes with an edge pixel coinciding
// 		gl_FragColor = green;
// 		return;
// 	}else{
// 		gl_FragColor = red;
// 		if(!analyze)
// 			discard;
// 		return;
// 	}
	
}

