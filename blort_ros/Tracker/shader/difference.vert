varying vec4 vertex_model;

void main(){
	vertex_model = gl_Vertex;
	gl_Position = ftransform();
	gl_TexCoord[0]  = gl_MultiTexCoord0;
}