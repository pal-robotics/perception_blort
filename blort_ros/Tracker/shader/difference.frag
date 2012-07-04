varying vec4 vertex_model;

uniform sampler2D tex_frame;
uniform sampler2D tex_model;

uniform mat4 modelviewprojection; // -matrix of previouse tracked position

uniform float fTol; // tolerance for angle comparison
uniform bool compare;
uniform bool analyze;

uniform vec4 drawcolor;

const float inf = 100000.0;
const float dpi = 0.318309886;  // dpi = 1 / pi;

const vec4 white	= vec4(1.0, 1.0, 1.0, 1.0);
const vec4 red 		= vec4(1.0, 0.0, 0.0, 0.0);
const vec4 green 	= vec4(0.0, 1.0, 0.0, 0.0);
const vec4 blue 	= vec4(0.0, 0.0, 1.0, 0.0);
const vec4 black 	= vec4(0.0, 0.0, 0.0, 0.0);

void main(){
	vec4 texcoords_model;
	vec4 texcoords_frame;
	vec4 color_frame;
	vec4 color_model;
	vec4 color_result;
	float difference;
	
	// calculate texture coordinats for forward-mapping
	texcoords_model.xy = gl_TexCoord[0].xy;
	
	// calculate texture coordinats for forward-mapping
	texcoords_frame = gl_ModelViewProjectionMatrix * vertex_model;
	texcoords_frame.x = (texcoords_frame.x / texcoords_frame.w + 1.0) * 0.5;
	texcoords_frame.y = (texcoords_frame.y / texcoords_frame.w + 1.0) * 0.5;	
	
	// get texture color for frame and model
	color_model = texture2D(tex_model, texcoords_model.xy);
	color_frame = texture2D(tex_frame, texcoords_frame.xy);
	
	color_result = color_frame-color_model;
	difference = (color_result.x + color_result.y + color_result.z) / 3.0;		
	difference = difference * 0.5 + 0.5;
	
	gl_FragColor = vec4(difference, 0.0, 0.0, 0.0);
}



