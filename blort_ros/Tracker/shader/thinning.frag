
// handle to texture
uniform sampler2D frame;
uniform sampler2D mask;

// pixel offsets (to load from program)
uniform mat3 mOffsetX;
uniform mat3 mOffsetY;

// threshold for compare statement
uniform float fThreshold = 0.0;

// enable thinning, or just normalize
uniform bool thinning_enabled = true;

uniform bool masked;

const vec4 vNull = vec4(0.5,0.5,0.0,0.0);
const float pi = 3.141592;

vec2 getOffset(vec2 vG){
    ivec2 vNb = ivec2(0,0);
    vec2 offset = vec2(0.0,0.0);
    
    // find pixel to gradient direction
    if(abs(vG.x) > abs(vG.y)){
        offset = vec2(mOffsetX[1][2],mOffsetY[1][2]);
    }else{
        offset = vec2(mOffsetX[0][1],mOffsetY[0][1]);
    }
    return offset;
}

vec4 thinning(){
    vec2 vG;
    vec3 vG_tmp;
    //vec2 vN;
    float fNm;
    vec2 offset;
    float fGm;
    float ft = fThreshold;
    float f_len_1, f_len_2;
    
    
    if(masked){
    	vec4 vColor;
		vColor = texture2D(mask, gl_TexCoord[0].st);
		if( vColor.z != 0.0 )
			return vNull;
	}
    
    // get edge gradient from texture
    vG_tmp = texture2D(frame, gl_TexCoord[0].st).xyz;
    fGm = vG_tmp.z;	// magnitude
    vG = (vG_tmp.xy - 0.5) * 2.0;  // scale to range [-1 ... 1]
    //vG = normalize(vG);
    
    // if pixel is not part of an edge then do nothing
    if(fGm < 0.001)
        return vNull;
    
    // get offset in texture coordinates for neighbouring pixel
    // in edge gradient direction
  	offset = getOffset(vG);
			
		// get color of neighbouring pixel
		fNm = texture2D(frame, gl_TexCoord[0].st + offset ).z;
		//vN = (vN - 0.5) * 2.0;  // scale to  [-1 ... 1]
		// remove this pixel if neighbouring pixel is stronger
		f_len_1 = fNm;
		if(fGm+ft < f_len_1) 
			return vNull;
		
		// Compare again for negative gradient direction 
		fNm = texture2D(frame, gl_TexCoord[0].st - offset ).z;
		//vN = (vN - 0.5) * 2.0;
		
		// remove this pixel if neighbouring pixel is stronger
		f_len_2 = fNm;
		if(fGm+ft < f_len_2) 
			return vNull;
				
			
		// remove this pixel if neighbouring pixels are no edges
		if( (f_len_1 < 0.001) && (f_len_2 < 0.001) )
			return vNull;
	
    // set magnitude to 1
    vG = normalize(vG);
    
    // scale to range [0 ... 1]
    vG = vG * 0.5 + 0.5;
    // return gradient (xy) and magnitude (z) and safe it as color
    return vec4(vG,fGm,0.0);
}

void main(){
    gl_FragColor = thinning();
}
