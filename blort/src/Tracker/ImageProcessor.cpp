#include <ros/console.h>

#include <blort/Tracker/ImageProcessor.h>
#include <blort/TomGine/tgError.h>

using namespace Tracking;


// Load and compile shaders and set parameters
bool ImageProcessor::initShader(){
	int id;
	float w = (float)m_width;
	float h = (float)m_height;
	float hi,lo;
	float sq2 = 1.0f/sqrt(2.0f);
	
	// offsets of neighbouring pixels in texture coordinates
	GLfloat offX[9] = { -1.0f/w, 0.0, 1.0f/w,
						-1.0f/w, 0.0, 1.0f/w,
						-1.0f/w, 0.0, 1.0f/w };
	GLfloat offY[9] = {  1.0f/h, 1.0f/h,  1.0f/h,
						 0.0,   0.0,    0.0,
						-1.0f/h,-1.0f/h, -1.0f/h };
	GLfloat dist[9] = { sq2, 1.0f, sq2,
						1.0f, 0.0f, 1.0f,
						sq2, 1.0f, sq2 };
	GLfloat kernel[25] = {	2,  4,  5,  4, 2,
							4,  9, 12,  9, 4,
							5, 12, 15, 12, 5,
							4,  9, 12,  9, 4,
							2,  4,  5,  4, 2 };
	hi = 10.0f/22; // = sqrt((3+10+3)^2 + (3+10+3)^2) = 22.6
	lo = 3.0f/22; 
	GLfloat sobelX[9] = {   -lo, 	0.0f, 	lo,
							-hi, 	0.0f, 	hi,
							-lo,  	0.0f, 	lo };
	// dont modify structure of sobelY -> division in sobel.frag
	GLfloat sobelY[9] = {    lo,	hi,     lo,
							 0.0f,	0.0f,	0.0f,
							-lo,   -hi,	    -lo };
	
	// Gauss shader
	id = g_Resources->AddShader("gauss", NULL, "gauss.frag");
	m_shadeGauss = g_Resources->GetShader(id);
	m_shadeGauss->bind();
	m_shadeGauss->setUniform( "kernel", 25, kernel );
	m_shadeGauss->setUniform( "width", w);
	m_shadeGauss->setUniform( "height", h);
	m_shadeGauss->unbind();
	
	// Sobel shader
	id = g_Resources->AddShader("sobel", NULL, "sobel.frag");
	m_shadeSobel = g_Resources->GetShader(id);
	m_shadeSobel->bind();
	m_shadeSobel->setUniform( "frame", 0 );
	m_shadeSobel->setUniform( "mask", 1);
	m_shadeSobel->setUniform( "binary", false);
	m_shadeSobel->setUniform( "mOffsetX", mat3(offX), GL_FALSE );
	m_shadeSobel->setUniform( "mOffsetY", mat3(offY), GL_FALSE );
	m_shadeSobel->setUniform( "mSobelX", mat3(sobelX), GL_FALSE );
	m_shadeSobel->setUniform( "mSobelY", mat3(sobelY), GL_FALSE );
	m_shadeSobel->setUniform( "fThreshold", 0.01f );
	m_shadeSobel->unbind();
	
	// Thinning shader
	id = g_Resources->AddShader("thinning", NULL, "thinning.frag");
	m_shadeThinning = g_Resources->GetShader(id);
	m_shadeThinning->bind();
	m_shadeThinning->setUniform( "frame", 0 );
	m_shadeThinning->setUniform( "mask", 1);
	m_shadeThinning->setUniform( "mOffsetX", mat3(offX), GL_FALSE );
	m_shadeThinning->setUniform( "mOffsetY", mat3(offY), GL_FALSE );
	m_shadeThinning->setUniform( "fThreshold", 0.0f );
	m_shadeThinning->unbind();
	
	// Spreading shader
	id = g_Resources->AddShader("spreading", NULL, "spreading.frag");
	m_shadeSpreading = g_Resources->GetShader(id);
	m_shadeSpreading->bind();
	m_shadeSpreading->setUniform( "mOffsetX", mat3(offX), GL_FALSE );
	m_shadeSpreading->setUniform( "mOffsetY", mat3(offY), GL_FALSE );
	m_shadeSpreading->setUniform( "mDistance", mat3(dist), GL_FALSE );
	m_shadeSpreading->setUniform( "fThreshold", 0.01f );
	m_shadeSpreading->setUniform( "fDistScale", 0.5f );
	m_shadeSpreading->unbind();
	
	return true;
}

// Display list for normal images (non-rectificating TexCoords)
bool ImageProcessor::dlImage(){
//     float x = float(m_width)*0.5f;
//     float y = float(m_height)*0.5f;
//     glBegin(GL_QUADS);
// 		glColor3f(1.0f,1.0f,1.0f);
//         glTexCoord2f(0,0); glVertex3f(-x,-y, 0.0f);
//         glTexCoord2f(1,0); glVertex3f( x,-y, 0.0f);
//         glTexCoord2f(1,1); glVertex3f( x, y, 0.0f);
//         glTexCoord2f(0,1); glVertex3f(-x, y, 0.0f);
//     glEnd();

    float x = float(m_width);
    float y = float(m_height);
    glBegin(GL_QUADS);
		glColor3f(1.0f,1.0f,1.0f);
        glTexCoord2f(0,0); glVertex3f( 0, 0, 0.0f);
        glTexCoord2f(1,0); glVertex3f( x, 0, 0.0f);
        glTexCoord2f(1,1); glVertex3f( x, y, 0.0f);
        glTexCoord2f(0,1); glVertex3f( 0, y, 0.0f);
    glEnd();
    
    return true;
}

bool ImageProcessor::dlImage(float x, float y, float w, float h){

	glBegin(GL_QUADS);
	glColor3f(1.0f,1.0f,1.0f);
			glTexCoord2f(0.0f,0.0f); glVertex3f(x,   y, 0.0f);
			glTexCoord2f(1.0f,0.0f); glVertex3f(x+w, y, 0.0f);
			glTexCoord2f(1.0f,1.0f); glVertex3f(x+w, y+h, 0.0f);
			glTexCoord2f(0.0f,1.0f); glVertex3f(x,   y+h, 0.0f);
	glEnd();
	
	return true;
}

// Display list for flipping image upside down
bool ImageProcessor::dlFlipUpsideDown(){
//     float x = float(m_width>>1);
//     float y = float(m_height>>1);
//     
//     glBegin(GL_QUADS);
// 		glTexCoord2f(0.0f,1.0f); glVertex3f(-x,-y, 0.0f);
//         glTexCoord2f(1.0f,1.0f); glVertex3f( x,-y, 0.0f);
//         glTexCoord2f(1.0f,0.0f); glVertex3f( x, y, 0.0f);
//         glTexCoord2f(0.0f,0.0f); glVertex3f(-x, y, 0.0f);
//     glEnd();
	
	float x = float(m_width);
    float y = float(m_height);
    
    glBegin(GL_QUADS);
		glTexCoord2f(0.0f,1.0f); glVertex3f( 0, 0, 0.0f);
        glTexCoord2f(1.0f,1.0f); glVertex3f( x, 0, 0.0f);
        glTexCoord2f(1.0f,0.0f); glVertex3f( x, y, 0.0f);
        glTexCoord2f(0.0f,0.0f); glVertex3f( 0, y, 0.0f);
    glEnd();
    
    return true;
}

// Display list for rectificating image (rectificating TexCoords)
bool ImageProcessor::dlRectification(){
    unsigned i,j,n=10;
    float x,y;
    unsigned w2 = m_width >> 1;
    unsigned h2 = m_height >> 1;
    
    // Lens rectificated texture coordinates
    glBegin(GL_QUADS);
        for (i=0;i<w2;i+=n) {
            for (j=0;j<h2;j+=n) {
				float fi = float(i);
				float fj = float(j);
                transform(fi,fj,&x,&y);
                glTexCoord2f(x,y);
                glVertex3f(fi-w2,fj-h2,0.0f);
                
                transform(fi+n,fj,&x,&y);
                glTexCoord2f(x,y);
                glVertex3f((fi+n)-w2,fj-h2,0.0f);
                
                transform(fi+n,fj+n,&x,&y);
                glTexCoord2f(x,y);
                glVertex3f((fi+n)-w2,(fj+n)-h2,0.0f);
                
                transform(fi,fj+n,&x,&y);
                glTexCoord2f(x,y);
                glVertex3f(fi-w2,(fj+n)-h2,0.0f);
            }
        } 
    glEnd();    
	return true;
}

// Gets rectificated texture coordinates ix, iy for pixel at position i,j
bool ImageProcessor::transform(float i, float j, float *ix, float *iy){
    float a,b,c,d;
    float x,y,xnew,ynew;
    float r,theta,rnew,thetanew;
    int w = m_width;
    int h = m_height;

    x = i / (w*0.5f) - 1;
    y = j / (h*0.5f) - 1;
    r = sqrt(x*x+y*y);
    theta = atan2(y,x);
    
    switch (m_lensMode) {
    case NONE:
        xnew = x;
        ynew = y;
        break;
    case BARREL:
        a = -0.005f;
        b = 0.0f;
        c = 0.0f;
        d = 1.0f;
        rnew = (a*r*r*r + b*r*r + c*r + d) * r;
        thetanew = theta;
        xnew = rnew * cos(thetanew);
        ynew = rnew * sin(thetanew);
        break;
    }
    *ix = (xnew + 1)*0.5f;
    *iy = (ynew + 1)*0.5f;
    
    return true;
}

ImageProcessor::ImageProcessor(){
	m_shadeSobel = 0;
	m_shadeThinning = 0;
	m_shadeSpreading = 0;
	m_sum_init = false;
}

ImageProcessor::~ImageProcessor(){
	glDeleteLists(m_dlRect, 1);
	glDeleteLists(m_dlImage, 1);
	glDeleteLists(m_dlUpsideDown, 1);
	if(m_sum_init){
		glDeleteFramebuffers(1, &fbo);
		glDeleteTextures(1,&fbo_tex);
		glDeleteTextures(1,&fbo_tex_depth);
	}
}

// Set functions
void ImageProcessor::setCamOrtho(){ 
	m_cam_ortho.Activate();
}

// *** Image Processing functions ***
void ImageProcessor::flipUpsideDown(Texture* source, Texture* result){
	m_cam_ortho.Activate();
	glEnable(GL_TEXTURE_2D);
		source->bind();
		glCallList(m_dlUpsideDown);
		result->copyTexImage2D(source->getWidth(), source->getHeight());
	glDisable(GL_TEXTURE_2D);
}

void ImageProcessor::copy(Texture* source, Texture* result){
	m_cam_ortho.Activate();
	glEnable(GL_TEXTURE_2D);
		source->bind();
		glCallList(m_dlImage);
    	result->copyTexImage2D(source->getWidth(), source->getHeight());
    glDisable(GL_TEXTURE_2D);
}

void ImageProcessor::rectification(Texture* source, Texture* result){
	m_cam_ortho.Activate();
	glEnable(GL_TEXTURE_2D);
		source->bind();
		glCallList(m_dlRect);
    	result->copyTexImage2D(source->getWidth(), source->getHeight());
    glDisable(GL_TEXTURE_2D);
}

void ImageProcessor::gauss(Texture* source, Texture* result){
	m_cam_ortho.Activate();
	m_shadeGauss->bind();
    glEnable(GL_TEXTURE_2D);
		source->bind();
		glCallList(m_dlImage);
    	result->copyTexImage2D(source->getWidth(), source->getHeight());
    glDisable(GL_TEXTURE_2D);
    m_shadeGauss->unbind();
}

void ImageProcessor::sobel(Texture* source, Texture* result, float threshold, bool normalise, bool binary){
	m_cam_ortho.Activate();
	m_shadeSobel->bind();
	m_shadeSobel->setUniform( "fThreshold", threshold );
	m_shadeSobel->setUniform( "norm", normalise);
	m_shadeSobel->setUniform( "binary", binary);
    glEnable(GL_TEXTURE_2D);
		source->bind();
		glCallList(m_dlImage);
    	result->copyTexImage2D(source->getWidth(), source->getHeight());
    glDisable(GL_TEXTURE_2D);
  m_shadeSobel->unbind();
}

void ImageProcessor::sobel(Texture* source, Texture* result, Texture* mask, float threshold, bool normalise, bool binary){
	m_cam_ortho.Activate();
	m_shadeSobel->bind();
	m_shadeSobel->setUniform( "fThreshold", threshold );
	m_shadeSobel->setUniform( "norm", normalise);
	m_shadeSobel->setUniform( "masked", true);
	m_shadeSobel->setUniform( "binary", binary);
    glEnable(GL_TEXTURE_2D);
		source->bind(0);
		mask->bind(1);
		glCallList(m_dlImage);
    	result->copyTexImage2D(source->getWidth(), source->getHeight());
    glDisable(GL_TEXTURE_2D);
  m_shadeSobel->unbind();
  m_shadeSobel->setUniform( "masked", false);
}

void ImageProcessor::thinning(Texture* source, Texture* result){
	m_cam_ortho.Activate();
	glEnable(GL_TEXTURE_2D);
		source->bind();
		m_shadeThinning->bind();
			glCallList(m_dlImage);
		m_shadeThinning->unbind();
		result->copyTexImage2D(source->getWidth(), source->getHeight());
    glDisable(GL_TEXTURE_2D);
}

void ImageProcessor::thinning(Texture* source, Texture* result, Texture* mask){
	m_cam_ortho.Activate();
	m_shadeThinning->bind();
	m_shadeThinning->setUniform( "masked", true);
	glEnable(GL_TEXTURE_2D);
		source->bind(0);
		mask->bind(1);
		glCallList(m_dlImage);
		result->copyTexImage2D(source->getWidth(), source->getHeight());
    glDisable(GL_TEXTURE_2D);
	m_shadeThinning->setUniform( "masked", false);
	m_shadeThinning->unbind();
}

void ImageProcessor::spreading(Texture* source, Texture* result){
	m_cam_ortho.Activate();
	glEnable(GL_TEXTURE_2D);
		source->bind();
		m_shadeSpreading->bind();
		glCallList(m_dlImage);
		m_shadeSpreading->unbind();
		result->copyTexImage2D(source->getWidth(), source->getHeight());
    glDisable(GL_TEXTURE_2D);;
}

void ImageProcessor::render(Texture* tex){
	m_cam_ortho.Activate();
	glEnable(GL_TEXTURE_2D);
		tex->bind();
		glCallList(m_dlImage);
	glDisable(GL_TEXTURE_2D);
}

void ImageProcessor::render(Texture* tex, int x, int y, unsigned w, unsigned h){
	m_cam_ortho.Activate();
	glEnable(GL_TEXTURE_2D);
		tex->bind();
		dlImage((float)x, (float)y, (float)w, (float)h);
	glDisable(GL_TEXTURE_2D);
}

GLenum ImageProcessor::avgInit(int res)
{
	if(res < 16)
                ROS_DEBUG("[ImageProcessor::avgInit] Warning: resolution too low\n"); // doesn't make sense computing low number of sums on GPU
	
	bool res_valid = false;
	for(unsigned i=4; i<13; i++)
		if(res == pow(2,i))
			res_valid = true;
	
	if(!res_valid)
                ROS_DEBUG("[ImageProcessor::avgInit] Warning: resolution must be power of 2 and in the range of 16 to 4048\n");
	
	fbo_res = res;
	
	
	glGenTextures(1, &fbo_tex_depth);
	glBindTexture(GL_TEXTURE_2D, fbo_tex_depth);
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR); 
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_GENERATE_MIPMAP, GL_TRUE); // for attaching to fbo texture must be mipmap complete
// 	glTexImage2D( GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT32, fbo_res, fbo_res, 0, GL_DEPTH_COMPONENT, GL_UNSIGNED_BYTE, NULL);
	
	// Texture (=memory) for fbo
	glGenTextures(1, &fbo_tex);
	glBindTexture(GL_TEXTURE_2D, fbo_tex);
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR); 
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_GENERATE_MIPMAP, GL_TRUE); // for attaching to fbo texture must be mipmap complete
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, fbo_res, fbo_res, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
	
	
	// fbo = framebuffer object
	glGenFramebuffers(1,&fbo);
	glBindFramebuffer(GL_FRAMEBUFFER, fbo);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, fbo_tex, 0);
// 	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, fbo_tex_depth, 0);
	
	m_cam_ortho_fbo.Set(	0.0f, 0.0f, 1.0f,
							0.0f, 0.0f, 0.0f,
							0.0f, 1.0f, 0.0f,
							45.0f, fbo_res, fbo_res,
							0.0f, 1.0f,
							GL_ORTHO);

	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	
	fbo_stage = ilog2(fbo_res);
	m_sum_init = true;
	
	tgCheckFBError(GL_FRAMEBUFFER, "ImageProcessor::avgInit()");
	return tgCheckError("ImageProcessor::avgInit()");
}

void ImageProcessor::avgActivate(){
	if(m_sum_init){
		glBindFramebuffer(GL_FRAMEBUFFER, fbo);
		tgCheckError("ImageProcessor::avgActivate() A");
		glClearColor(0,0,0,0);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		tgCheckError("ImageProcessor::avgActivate() B");
	}
}

void ImageProcessor::avgGet(float *avg, int lvl){
	if(m_sum_init){
		if(lvl==0){
			// for some reason glGetTexImage doesen't get the value of the very last mipmap stage
// 			float tmp[16];
// 			glBindTexture(GL_TEXTURE_2D, fbo_tex);
// 			glGenerateMipmap(GL_TEXTURE_2D);
// 			glGetTexImage(GL_TEXTURE_2D, fbo_stage-1, GL_RGBA, GL_FLOAT, tmp);
// 			avg[0] = 0;
// 			for(unsigned i=0; i<4; i++)
// 				avg[0] += 0.25 * tmp[i];
                        ROS_DEBUG("[ImageProcessor::avgGet] Warning: not implemented\n");
		}else{
			glBindTexture(GL_TEXTURE_2D, fbo_tex);
// 			glCopyTexImage2D(GL_TEXTURE_2D, 0, GL_RGB16,0, 0, fbo_res,fbo_res,0);
			glGenerateMipmap(GL_TEXTURE_2D);
			glGetTexImage(GL_TEXTURE_2D, fbo_stage-lvl, GL_RED, GL_FLOAT, avg);
			tgCheckError("ImageProcessor::avgGet()");
		}
	}
}

void ImageProcessor::avgDeactivate(){
	if(m_sum_init){
		glBindFramebuffer(GL_FRAMEBUFFER, 0);
	}
}

// Main initialisation function
bool ImageProcessor::init(unsigned w, unsigned h){
    
	m_width = w;
	m_height = h;
	
	// Initialise camera
	m_cam_ortho.Set(	0.0f, 0.0f, 1.0f,
						0.0f, 0.0f, 0.0f,
						0.0f, 1.0f, 0.0f,
						45.0f, w, h,
						0.1f, 10.0f,
						GL_ORTHO);
	
	// Initialize shaders
	if(!initShader())
		return false;
    
    
	// Setup display lists
	m_dlRect = glGenLists(1);
	m_lensMode = BARREL;
	glNewList(m_dlRect, GL_COMPILE);
 	dlRectification();
	glEndList();
	
	m_dlImage = glGenLists(1);
	glNewList(m_dlImage, GL_COMPILE);
	dlImage();
	glEndList();
	
	m_dlUpsideDown = glGenLists(1);
	glNewList(m_dlUpsideDown, GL_COMPILE);
		dlFlipUpsideDown();
	glEndList();
	
	return true;
}


