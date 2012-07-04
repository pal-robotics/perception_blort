
uniform sampler2D frame;

uniform mat3 mOffsetX;
uniform mat3 mOffsetY;
uniform mat3 mDistance;

uniform float fThreshold;
uniform float fDistScale;

vec4 spreading(){
	vec4 vColor, vNeighbour;
	vec4 vMaxNeighbour = vec4(0.0, 0.0, 0.0, 0.0);
	float fMag;
	float fDist;
	
	// get gradient and magnitude of pixel stored in color
	vColor = texture2D(frame, gl_TexCoord[0]);
	
	// if pixel is an edge pixel, do nothing
	if(vColor.z > 0.01){
		return vColor;
	}
	
	// parse through neighbouring pixels
	for(int i=0; i<3; i++){
		for(int j=0; j<3; j++){
			if( (i==1 || j==1) && (i!=j) ){
				// Erode filter:
				// - * -
				// * - *
				// - * -
			
				vNeighbour = texture2D(frame, gl_TexCoord[0].st + vec2(mOffsetX[i][j], mOffsetY[i][j]));
				// if magnitude of adjacent pixel is bigger than a threshold
				// set magnitude of this pixel to that of the adjacent pixel considering distance
				if(vNeighbour.z > fThreshold){
					// only pass gradient and magnitude of neighbour with maximum magnitude
					if(vNeighbour.z > vMaxNeighbour.z){
						vMaxNeighbour = vNeighbour;
						fDist = fDistScale * mDistance[i][j];   // distance scaling
						fMag = vMaxNeighbour.z * fDist;
						vColor = vec4(vMaxNeighbour.xy, fMag, 0.0);
					}
				}
			}
		}
	}
	
	return vColor;
}

void main(){
    gl_FragColor = spreading();
}
