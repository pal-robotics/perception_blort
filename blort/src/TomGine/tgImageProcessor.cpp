
#include <blort/TomGine/tgImageProcessor.h>
#include <blort/TomGine/tgError.h>
#include <stdexcept>

using namespace TomGine;

tgImageProcessor::tgImageProcessor(	const char *gauss_frag_file,
									const char *sobel_frag_file,
									const char *thinning_frag_file,
									const char *spreading_frag_file,
									unsigned img_width, unsigned img_height,
									int avg_resolution)
{
	m_shadeGauss = new tgShader(NULL, gauss_frag_file, NULL);
	m_shadeSobel = new tgShader(NULL, sobel_frag_file, NULL);
	m_shadeThinning = new tgShader(NULL, thinning_frag_file, NULL);
	m_shadeSpreading = new tgShader(NULL, spreading_frag_file, NULL);
	
	init(img_width, img_height);
	initShader((float)img_width, (float)img_height);
	initFBO(avg_resolution);
	tgCheckError("[tgImageProcessor::tgImageProcessor()]");
}

tgImageProcessor::~tgImageProcessor(){
	if(glIsFramebuffer(fbo)) glDeleteFramebuffers(1, &fbo);
	if(glIsTexture(fbo_tex)) glDeleteTextures(1,&fbo_tex);
	if(glIsTexture(fbo_tex_depth)) glDeleteTextures(1,&fbo_tex_depth);

	if(m_shadeGauss) delete(m_shadeGauss);
	if(m_shadeSobel) delete(m_shadeSobel);
	if(m_shadeThinning) delete(m_shadeThinning);
	if(m_shadeSpreading) delete(m_shadeSpreading);
}

// Main initialisation function
void tgImageProcessor::init(unsigned width, unsigned height){
    
	m_width = width;
	m_height = height;

	// Initialise camera
	m_cam_ortho.Set(	0.0f, 0.0f, 1.0f,
						0.0f, 0.0f, 0.0f,
						0.0f, 1.0f, 0.0f,
						45.0f, m_width, m_height,
						0.1f, 10.0f,
						GL_ORTHO);
}

// Load and compile shaders and set parameters
void tgImageProcessor::initShader(float w, float h){
	float sq2 = 1.0f/sqrt(2.0f);
	
	// offsets of neighbouring pixels in texture coordinates
	GLfloat offX[9] = { -1.0f/w, 0.0, 1.0f/w,
						-1.0f/w, 0.0, 1.0f/w,
						-1.0f/w, 0.0, 1.0f/w };
	GLfloat offY[9] = {  1.0f/h, 1.0f/h,  1.0f/h,
						 0.0,   0.0,    0.0,
						-1.0f/h,-1.0f/h, -1.0f/h };
	
	// distance of neighbouring pixels
	GLfloat dist[9] = { sq2, 1.0f, sq2,
						1.0f, 0.0f, 1.0f,
						sq2, 1.0f, sq2 };
	GLfloat kernel[25] = {	2,  4,  5,  4, 2,
							4,  9, 12,  9, 4,
							5, 12, 15, 12, 5,
							4,  9, 12,  9, 4,
							2,  4,  5,  4, 2 };
	float hi = 10.0f/22; // = sqrt((3+10+3)^2 + (3+10+3)^2) = 22.6
	float lo = 3.0f/22; 
	GLfloat sobelX[9] = {   -lo, 	0.0f, 	lo,
							-hi, 	0.0f, 	hi,
							-lo,  	0.0f, 	lo };
	// dont modify structure of sobelY -> division in sobel.frag
	GLfloat sobelY[9] = {    lo,	hi,     lo,
							 0.0f,	0.0f,	0.0f,
							-lo,   -hi,	    -lo };
	
	// Gauss shader
	m_shadeGauss->bind();
	m_shadeGauss->setUniform( "kernel", 25, kernel );
	m_shadeGauss->setUniform( "width", w);
	m_shadeGauss->setUniform( "height", h);
	m_shadeGauss->unbind();
	
	// Sobel shader
	m_shadeSobel->bind();
	m_shadeSobel->setUniform( "frame", 0 );
	m_shadeSobel->setUniform( "mask", 1);
	m_shadeSobel->setUniform( "binary", false);
	m_shadeSobel->setUniform( "mOffsetX", mat3(offX), GL_FALSE );
	m_shadeSobel->setUniform( "mOffsetY", mat3(offY), GL_FALSE );
	m_shadeSobel->setUniform( "mSobelX", mat3(sobelX), GL_FALSE );
	m_shadeSobel->setUniform( "mSobelY", mat3(sobelY), GL_FALSE );
	m_shadeSobel->setUniform( "fThreshold", 0.1f );
	m_shadeSobel->unbind();
	
	// Thinning shader
	m_shadeThinning->bind();
	m_shadeThinning->setUniform( "frame", 0 );
	m_shadeThinning->setUniform( "mask", 1);
	m_shadeThinning->setUniform( "mOffsetX", mat3(offX), GL_FALSE );
	m_shadeThinning->setUniform( "mOffsetY", mat3(offY), GL_FALSE );
	m_shadeThinning->setUniform( "fThreshold", 0.1f );
	m_shadeThinning->unbind();
	
	// Spreading shader
	m_shadeSpreading->bind();
	m_shadeSpreading->setUniform( "mOffsetX", mat3(offX), GL_FALSE );
	m_shadeSpreading->setUniform( "mOffsetY", mat3(offY), GL_FALSE );
	m_shadeSpreading->setUniform( "mDistance", mat3(dist), GL_FALSE );
	m_shadeSpreading->setUniform( "fThreshold", 0.1f );
	m_shadeSpreading->setUniform( "fDistScale", 0.5f );
	m_shadeSpreading->unbind();
}

void tgImageProcessor::initFBO(int res)
{
	if(res < 16)
		printf("[tgImageProcessor::avgInit] Warning: resolution too low\n"); // doesn't make sense computing low number of sums on GPU
	
	bool res_valid = false;
	for(unsigned i=4; i<13; i++)
		if(res == pow(2,i))
			res_valid = true;
	
	if(!res_valid)
		printf("[tgImageProcessor::avgInit] Warning: resolution must be power of 2 and in the range of 16 to 4048\n");
	
	fbo_res = res;
	
	glGenTextures(1, &fbo_tex_depth);
	glBindTexture(GL_TEXTURE_2D, fbo_tex_depth);
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR); 
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_GENERATE_MIPMAP, GL_TRUE); // for attaching to fbo texture must be mipmap complete
// 	glTexImage2D( GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT32, fbo_res, fbo_res, 0, GL_DEPTH_COMPONENT, GL_UNSIGNED_BYTE, NULL);
	
	// tgTexture (=memory) for fbo
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
// 	glFramebuffertgTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, fbo_tex_depth, 0);
	
	m_cam_ortho_fbo.Set(	0.0f, 0.0f, 1.0f,
							0.0f, 0.0f, 0.0f,
							0.0f, 1.0f, 0.0f,
							45.0f, fbo_res, fbo_res,
							0.0f, 1.0f,
							GL_ORTHO);

	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	
	fbo_stage = ilog2(fbo_res);
	m_avg_init = true;
	
	if(	tgCheckFBError(GL_FRAMEBUFFER, "tgImageProcessor::avgInit()")!=GL_FRAMEBUFFER_COMPLETE ||
		tgCheckError("tgImageProcessor::avgInit()")!=GL_NO_ERROR)
	{
		std::string errmsg = std::string("[tgImageProcessor::initFBO()] Error generating frame buffer objects");
		throw std::runtime_error(errmsg.c_str());
	}
}

void tgImageProcessor::drawQuad(float w, float h){    
	glBegin(GL_QUADS);
		glTexCoord2f(0.0f,0.0f); glVertex3f( 0, 0, 0.0f);
		glTexCoord2f(1.0f,0.0f); glVertex3f( w, 0, 0.0f);
		glTexCoord2f(1.0f,1.0f); glVertex3f( w, h, 0.0f);
		glTexCoord2f(0.0f,1.0f); glVertex3f( 0, h, 0.0f);
	glEnd();
}

// Display list for flipping image upside down
void tgImageProcessor::drawQuadUpsideDown(float w, float h){    
	glBegin(GL_QUADS);
		glTexCoord2f(0.0f,1.0f); glVertex3f( 0, 0, 0.0f);
		glTexCoord2f(1.0f,1.0f); glVertex3f( w, 0, 0.0f);
		glTexCoord2f(1.0f,0.0f); glVertex3f( w, h, 0.0f);
		glTexCoord2f(0.0f,0.0f); glVertex3f( 0, h, 0.0f);
	glEnd();
}

// Set functions
void tgImageProcessor::setCamOrtho(){ 
	m_cam_ortho.Activate();
}

// *** Image Processing functions ***
void tgImageProcessor::flipUpsideDown(const tgTexture& source, tgTexture& result){
	int w = source.GetWidth();
	int h = source.GetHeight();
	m_cam_ortho.Activate();
	glEnable(GL_TEXTURE_2D);
		source.Bind();
		drawQuadUpsideDown(w, h);
		result.CopyTexImage2D(w, h);
	glDisable(GL_TEXTURE_2D);
}

void tgImageProcessor::copy(const tgTexture& source, tgTexture& result){
	int w = source.GetWidth();
	int h = source.GetHeight();
	m_cam_ortho.Activate();
	glEnable(GL_TEXTURE_2D);
		source.Bind();
		drawQuad(w,h);
		result.CopyTexImage2D(w, h);
	glDisable(GL_TEXTURE_2D);
}

void tgImageProcessor::gauss(const tgTexture& source, tgTexture& result){
	int w = source.GetWidth();
	int h = source.GetHeight();
	m_cam_ortho.Activate();
	m_shadeGauss->bind();
	glEnable(GL_TEXTURE_2D);
		source.Bind();
		drawQuad(w,h);
		result.CopyTexImage2D(w, h);
	glDisable(GL_TEXTURE_2D);
	m_shadeGauss->unbind();
}

void tgImageProcessor::sobel(const tgTexture& source, tgTexture& result, float threshold, bool normalise, bool binary){
	int w = source.GetWidth();
	int h = source.GetHeight();
	m_cam_ortho.Activate();
	m_shadeSobel->bind();
	m_shadeSobel->setUniform( "fThreshold", threshold );
	m_shadeSobel->setUniform( "norm", normalise);
	m_shadeSobel->setUniform( "binary", binary);
    glEnable(GL_TEXTURE_2D);
		source.Bind();
		drawQuad(w,h);
		result.CopyTexImage2D(w, h);
	glDisable(GL_TEXTURE_2D);
	m_shadeSobel->unbind();
}

void tgImageProcessor::sobel(const tgTexture& source, tgTexture& result, tgTexture& mask, float threshold, bool normalise, bool binary){
	int w = source.GetWidth();
	int h = source.GetHeight();
	m_cam_ortho.Activate();
	m_shadeSobel->bind();
	m_shadeSobel->setUniform( "fThreshold", threshold );
	m_shadeSobel->setUniform( "norm", normalise);
	m_shadeSobel->setUniform( "masked", true);
	m_shadeSobel->setUniform( "binary", binary);
	glEnable(GL_TEXTURE_2D);
		source.Bind(0);
		mask.Bind(1);
		drawQuad(w,h);
		result.CopyTexImage2D(w, h);
	glDisable(GL_TEXTURE_2D);
	m_shadeSobel->unbind();
	m_shadeSobel->setUniform( "masked", false);
}

void tgImageProcessor::thinning(const tgTexture& source, tgTexture& result){
	int w = source.GetWidth();
	int h = source.GetHeight();
	m_cam_ortho.Activate();
	m_shadeThinning->bind();
	glEnable(GL_TEXTURE_2D);
		source.Bind();
		drawQuad(w,h);
    	result.CopyTexImage2D(w, h);
    glDisable(GL_TEXTURE_2D);
	m_shadeThinning->unbind();
}

void tgImageProcessor::thinning(const tgTexture& source, tgTexture& result, tgTexture& mask){
	int w = source.GetWidth();
	int h = source.GetHeight();
	m_cam_ortho.Activate();
	m_shadeThinning->bind();
	m_shadeThinning->setUniform( "masked", true);
	glEnable(GL_TEXTURE_2D);
		source.Bind(0);
		mask.Bind(1);
		drawQuad(w,h);
    	result.CopyTexImage2D(w, h);
    glDisable(GL_TEXTURE_2D);
	m_shadeThinning->setUniform( "masked", false);
	m_shadeThinning->unbind();
}

void tgImageProcessor::spreading(const tgTexture& source, tgTexture& result){
	int w = source.GetWidth();
	int h = source.GetHeight();
	m_cam_ortho.Activate();
	m_shadeSpreading->bind();
	glEnable(GL_TEXTURE_2D);
		source.Bind();
		drawQuad(w,h);
    	result.CopyTexImage2D(w, h);
    glDisable(GL_TEXTURE_2D);
	m_shadeSpreading->unbind();
}

void tgImageProcessor::render(const tgTexture& tex){
	m_cam_ortho.Activate();
	glEnable(GL_TEXTURE_2D);
		tex.Bind();
		drawQuad(tex.GetWidth(), tex.GetHeight());
	glDisable(GL_TEXTURE_2D);
}

void tgImageProcessor::avgActivate(){
	if(m_avg_init){
		glBindFramebuffer(GL_FRAMEBUFFER, fbo);
		tgCheckError("tgImageProcessor::avgActivate() A");
		glClearColor(0,0,0,0);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		tgCheckError("tgImageProcessor::avgActivate() B");
	}
}

void tgImageProcessor::avgGet(float *avg, int lvl){
	if(m_avg_init){
		if(lvl==0){
			// for some reason glGetTexImage doesen't get the value of the very last mipmap stage
// 			float tmp[16];
// 			glBindtgTexture(GL_TEXTURE_2D, fbo_tex);
// 			glGenerateMipmap(GL_TEXTURE_2D);
// 			glGetTexImage(GL_TEXTURE_2D, fbo_stage-1, GL_RGBA, GL_FLOAT, tmp);
// 			avg[0] = 0;
// 			for(unsigned i=0; i<4; i++)
// 				avg[0] += 0.25 * tmp[i];
			printf("[tgImageProcessor::avgGet] Warning: not implemented\n");
		}else{
			glBindTexture(GL_TEXTURE_2D, fbo_tex);
// 			glCopyTexImage2D(GL_TEXTURE_2D, 0, GL_RGB16,0, 0, fbo_res,fbo_res,0);
			glGenerateMipmap(GL_TEXTURE_2D);
			glGetTexImage(GL_TEXTURE_2D, fbo_stage-lvl, GL_RED, GL_FLOAT, avg);
			tgCheckError("tgImageProcessor::avgGet()");
		}
	}
}

void tgImageProcessor::avgDeactivate(){
	if(m_avg_init){
		glBindFramebuffer(GL_FRAMEBUFFER, 0);
	}
}




