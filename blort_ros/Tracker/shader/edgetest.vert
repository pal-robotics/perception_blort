varying vec4 vertex_model;
varying vec2 vEdgeDirection;

void main(){
	vec4 vEDtmp;
	vertex_model = gl_Vertex;
	// Transform normal to projection plane
	vEDtmp = gl_ModelViewProjectionMatrix * vec4(gl_Normal,0.0);
	
	vEdgeDirection.x = -vEDtmp.y;
	vEdgeDirection.y = vEDtmp.x;
	
	// Normalize projection of normal
	vEdgeDirection = normalize(vEdgeDirection);
	// Forward to fragment shader
	gl_Position = ftransform();
}
