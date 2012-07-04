varying vec4 vertex;

void main(){
	vertex = gl_Vertex;
	gl_Position = ftransform();
	gl_TexCoord[0]  = gl_MultiTexCoord0;
}