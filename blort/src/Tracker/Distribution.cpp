#include <ros/console.h>
#include <blort/Tracker/Distribution.h>
#include <blort/TomGine/tgError.h>

using namespace Tracking;

// *** private ***
void Distribution::updateQueries(){
	
	if(queryMatches.empty()){
		queryMatches.assign(m_particlelist.size(), 0);
		glGenQueriesARB(m_particlelist.size(), &queryMatches[0]);
	}
		
	if(queryEdges.empty()){
		queryEdges.assign(m_particlelist.size(), 0);
		glGenQueriesARB(m_particlelist.size(), &queryEdges[0]);
	}
	
	if(	m_particlelist.size() != queryMatches.size() ){
	 glDeleteQueriesARB(queryMatches.size(), &queryMatches[0]);
	 queryMatches.resize(m_particlelist.size(), 0);
	 glGenQueriesARB(m_particlelist.size(), &queryMatches[0]);
	}
	 
	if( m_particlelist.size() != queryEdges.size() ){
		glDeleteQueriesARB(queryEdges.size(), &queryEdges[0]);
		queryEdges.resize(m_particlelist.size(), 0);
		glGenQueriesARB(m_particlelist.size(), &queryEdges[0]);
	}
}

void Distribution::calcMean(){
	m_meanParticle = Particle(0.0f);
	vec3 maxAxis = vec3(0.0f, 0.0f, 0.0f);
	float maxAngle = 0.0f;
	vec3 axis;
	float angle;
	w_sum = 0.0f;
	c_mean = 0.0f;
	int id;
	
	int num_particles = m_particlelist.size()>>1;
	
	// Normalise particles
	for(id=0; id<num_particles; id++)
		w_sum += m_particlelist[id].w;

	for(id=0; id<num_particles && w_sum > 0.0; id++)
		m_particlelist[id].w = m_particlelist[id].w / w_sum;
	
	// Weighted sum over all particles
	for(id=0; id<num_particles; id++){
// 		m_meanParticle.r  += m_particlelist[id].r * m_particlelist[id].w;
		m_meanParticle.t += m_particlelist[id].t * m_particlelist[id].w;
		c_mean 	+= m_particlelist[id].c;
		
		m_particlelist[id].q.getAxisAngle(axis, angle);
		maxAxis += axis * m_particlelist[id].w;
		maxAngle += angle * m_particlelist[id].w;
	}
	m_meanParticle.q.fromAxis(maxAxis, maxAngle);

	if(!m_particlelist.empty())
		c_mean = c_mean / num_particles;
	
	m_meanParticle.c = c_mean;
	m_meanParticle.w = w_max;
}

// *** public ***
Distribution::Distribution(){
	v_max = 1;
	w_sum = 0.0f;
}

Distribution::~Distribution(){
	if(!queryMatches.empty())
		glDeleteQueriesARB(m_particlelist.size(), &queryMatches[0]);
	if(!queryEdges.empty())
		glDeleteQueriesARB(m_particlelist.size(), &queryEdges[0]);
}

double Distribution::getVarianceC(){
	double mean = 0.0;
	double var = 0.0;
	
	// Evaluate mean
	for(unsigned id=0; id<m_particlelist.size(); id++)
		mean += m_particlelist[id].c;
	mean = mean / m_particlelist.size();
	
	// Evaluate standard variation
	for(unsigned id=0; id<m_particlelist.size(); id++)
		var += pow(m_particlelist[id].c - mean, 2);
	var = var / m_particlelist.size();
	
	return var;
}

double Distribution::getVariancePt(){
	double var = 0.0;
	
	// Evaluate mean
	Particle mean = getMean();
	
	// Evaluate standard variation
	for(unsigned id=0; id<m_particlelist.size(); id++){
		vec3 d = m_particlelist[id].t - mean.t;
		var += (d.x*d.x + d.y*d.y + d.z*d.z);
	}
	var = sqrt(var / m_particlelist.size());
	
	return var;
}

void Distribution::normalizeW(){
	float dw_sum = 0.0;
	if(w_sum>0.0){
		dw_sum = 1.0f/w_sum;
		for(unsigned id=0; id<m_particlelist.size(); id++){
			m_particlelist[id].w = m_particlelist[id].w * dw_sum;
			if(m_particlelist[id].w > w_max)
				w_max = m_particlelist[id].w;
		}
	}else{
		dw_sum = 1.0f/m_particlelist.size();
		for(unsigned id=0; id<m_particlelist.size(); id++){
			m_particlelist[id].w = dw_sum;
		}
		w_max = dw_sum;
	}
}

// Measurement
bool sortfunction(Particle p1, Particle p2){ return (p1.w > p2.w); }

void Distribution::drawParticlesEdges(TrackerModel& model, Shader* shadeCompare, bool showparticles){
	model.setTexture(0);
	glEnable(GL_DEPTH_TEST);
	
	for(unsigned i=0; i<m_particlelist.size(); i++){
		m_particlelist[i].Activate();
		
		glColorMask(0,0,0,0); glDepthMask(1);
		glClear(GL_DEPTH_BUFFER_BIT);
		model.drawFaces();
		
		glDepthMask(0);
		if(showparticles)
			glColorMask(1,1,1,1);
		
		glBeginQueryARB(GL_SAMPLES_PASSED_ARB, queryEdges[i]);
		model.drawEdges();
		glEndQueryARB(GL_SAMPLES_PASSED_ARB);
				
		glColorMask(0,0,0,0);
		shadeCompare->bind();
		shadeCompare->setUniform("analyze", false);
		glBeginQueryARB(GL_SAMPLES_PASSED_ARB, queryMatches[i]);
		model.drawEdges();
		glEndQueryARB(GL_SAMPLES_PASSED_ARB);
		shadeCompare->unbind();
		
		m_particlelist[i].Deactivate();
	}
	// Reset render masks
	glColorMask(1,1,1,1); glDepthMask(1);
}

void Distribution::drawParticlesTextured(TrackerModel& model, Shader* shader, bool showparticles)
{
	m_particlelist.size();
	
	// Set perspective mode and smaller viewport
	glColorMask(0,0,0,0); glDepthMask(0);
	glEnable(GL_DEPTH_TEST);
		
	// Draw particles and count pixels
	shader->bind();
	shader->setUniform("analyze", false);
	for(unsigned i=0; i<m_particlelist.size(); i++){
		m_particlelist[i].Activate();
		
		// Draw all model edge pixels
		glBeginQueryARB(GL_SAMPLES_PASSED_ARB, queryEdges[i]);
		shader->setUniform("compare", false);
		if(showparticles)
			glColorMask(1,1,1,1);
		model.drawTexturedFaces();
		model.drawUntexturedFaces();
		glEndQueryARB(GL_SAMPLES_PASSED_ARB);
		
		glColorMask(0,0,0,0);
		
		// Draw matching model edge pixels using a shader
		glBeginQueryARB(GL_SAMPLES_PASSED_ARB, queryMatches[i]);
		shader->setUniform("compare", true);
		shader->setUniform("textured", true);
		model.drawTexturedFaces();
		shader->setUniform("textured", false);
		model.drawUntexturedFaces();
		glEndQueryARB(GL_SAMPLES_PASSED_ARB);

		m_particlelist[i].Deactivate();
	}
	shader->unbind();
	glColorMask(1,1,1,1); glDepthMask(1);	
}

float Distribution::confidenceFunction(const float &m, const float &e)
{
	float c = 0.0f;
	
	if( e!=0 && v_max!=0 ){
		// TODO evaluate weights and convergence factor (w.r.t. robustness / accuracy / anti-locking)
		// TODO TODO TODO !!! HARDCODING !!!
		c = (0.8f * float(m)/float(e) + 0.2f * float(m)/float(v_max));
// 		c = ( 0.5f * float(d)/float(v) + 0.5f * float(d)/float(v_max));
// 		c = (1.0f * float(d)/float(v) + 0.0f * float(d)/float(v_max));
// 		c = (float(d)/float(v));
	}else{
		c = 0.0;
	}
	
	return c;
}

void Distribution::calcLikelihood(int convergence){
	int v, d;
	int v_max_tmp = 0;
	w_sum = 0.0;
	c_max = 0.0;
	w_max = 0.0;
	c_min = 1.0;
	
	for(unsigned id=0; id<m_particlelist.size(); id++){
		// Get number of pixels from Occlusion Query
		glGetQueryObjectivARB(queryEdges[id], GL_QUERY_RESULT_ARB, &v);
		glGetQueryObjectivARB(queryMatches[id], GL_QUERY_RESULT_ARB, &d);
		
		// Get maximum edge pixels (view dependent)
		if(v>v_max_tmp)
			v_max_tmp = v;
		
		// avoids  (d/v_max) > 1.0
		if(v>v_max)
			v_max = v;
		
		// Likelihood calculation formula
		m_particlelist[id].c = confidenceFunction(d, v);
		m_particlelist[id].w = pow(m_particlelist[id].c, float(convergence)*(1.0f-m_particlelist[id].c));
		
		if(m_particlelist[id].c > c_max)
			c_max = m_particlelist[id].c;
		
		if(m_particlelist[id].c < c_min)
			c_min = m_particlelist[id].c;
		
		// sum of likelihood over all particles
		w_sum += m_particlelist[id].w;
	}	
	
	// Update v_max (in case of decreasing v_max_tmp)
	v_max = v_max_tmp;
	
	if(c_max<=0.0){
		for(unsigned id=0; id<m_particlelist.size(); id++){
			m_particlelist[id].c = 0.01f;
		}
		c_max=0.01f;
	}
}

void Distribution::updateLikelihood(TrackerModel& model, Shader* shadeCompare, bool textured, int convergence, bool showparticles){
	
	// no particles to update
	if(m_particlelist.empty()){
		return;
	}
		
	updateQueries();
	
	if(textured)
		drawParticlesTextured(model, shadeCompare, showparticles);
	else
		drawParticlesEdges(model, shadeCompare, showparticles);
	
// 	printf("Distribution::updateLikelihood A: %f p: ", m_particlelist[0].c); m_particlelist[0].Print();
	
	calcLikelihood(convergence);
	
	// normalize weights
	normalizeW();

// 	printf("Distribution::updateLikelihood B: %f p: ", m_particlelist[0].c); m_particlelist[0].Print();
	
	// sort particles by likelihood
        std::stable_sort(m_particlelist.begin(), m_particlelist.end(), sortfunction);
	
// 	printf("Distribution::updateLikelihood C: %f p: ", m_particlelist[0].c); m_particlelist[0].Print();
	
	// calculate mean of distribution
	calcMean();
}

void Distribution::updateLikelihood(TrackerModel& model, Shader* shadeCompare, TomGine::tgCamera* cam_persp, Texture* tex_edge, int res)
{
	if(m_particlelist.empty()){
		return;
	}
	
	updateQueries();
	
	ImageProcessor *ip = g_Resources->GetImageProcessor();
	
	TomGine::tgPose p = getMean();
	int minX, maxX, minY, maxY;
	int fbo_res = ip->avgGetResolution();
	int segs = fbo_res / res;
	
	model.getBoundingBox2D( ip->getWidth(), ip->getHeight(), p, cam_persp, minX, maxX, minY, maxY );
	
	int w2 = ip->getWidth() >> 1;
	int h2 = ip->getHeight() >> 1;
	
	int width = 1.2 * (maxX-minX); // TODO hardcoded error tolerance (variance)
	int height = 1.2 * (maxY-minY); // TODO hardcoded error tolerance (variance)
	
	int minXsq = minX - ((width-(maxX-minX))>>1);
	int minYsq = minY - ((height-(maxY-minY))>>1);
	if(minXsq < 0) minXsq = 0;
	if(minYsq < 0) minYsq = 0;

	ip->setCamOrtho();
	glColor3f(1,1,1);
	glBegin(GL_LINE_LOOP);
		glTexCoord2f(0,0); glVertex3f(minX-w2,minY-h2,0);
		glTexCoord2f(1,0); glVertex3f(maxX-w2,minY-h2,0);
		glTexCoord2f(1,1); glVertex3f(maxX-w2,maxY-h2,0);
		glTexCoord2f(0,1); glVertex3f(minX-w2,maxY-h2,0);
	glEnd();
	
	cam_persp->Activate();
	
	p.Activate();
	glColor3f(1,0,0);
	model.drawEdges();
	p.Deactivate();
	
	float fResW = float(res)/width;
	float fResH = float(res)/height;
	
	unsigned steps = (unsigned)ceil(float(m_particlelist.size())/(segs*segs));
// 	printf("%d %d %d\n", m_particlelist.size(), segs, steps);
	
	float* avgs = (float*)malloc(sizeof(float)*segs*segs);
	int* pxs = (int*)malloc(sizeof(int)*segs*segs);
        for(int i=0; i<segs*segs; i++){
		avgs[i] = -1;
		pxs[i] = 0;
	}
	unsigned i=0;
	shadeCompare->bind();
	shadeCompare->setUniform("analyze", false);
	for(unsigned s=0; s<steps; s++){
		unsigned j=i;
		
// 		ip->avgActivate();
		tex_edge->bind(0);
                for(int r=0; r<segs; r++){
                        for(int c=0; c<segs; c++){
				if(i<m_particlelist.size()){
					glViewport(res*c-minXsq*fResW,res*r-minYsq*fResH, ip->getWidth()*fResW, ip->getHeight()*fResH);
				
					m_particlelist[i].Activate();
					glBeginQueryARB(GL_SAMPLES_PASSED_ARB, queryEdges[i]);
					
					shadeCompare->setUniform("compare", true);
					shadeCompare->setUniform("textured", true);
					model.drawTexturedFaces();
					shadeCompare->setUniform("textured", false);
					model.drawUntexturedFaces();
					
					glEndQueryARB(GL_SAMPLES_PASSED_ARB);
					m_particlelist[i].Deactivate();
				}
				i++;
			}
		}
		ip->avgGet(avgs, 2);
		ip->avgDeactivate();
		
		unsigned k=j;
		float c;
		float w;
// 		w_sum = 0.0;
// 		c_max = 0.0;
// 		w_max = 0.0;
// 		c_min = 1.0;
                for(int i=0; i<segs*segs; i++){
			if(k<m_particlelist.size()){
				glGetQueryObjectivARB(queryEdges[k], GL_QUERY_RESULT_ARB, &pxs[i]);
				tgCheckError("Distribustion::updateLikelihood glGetQueryObjectivARB: ");
				
				
				c = avgs[i]*float(res*res)/pxs[i];
				w = pow(c,3.0*(1.0f-c));
				
// 				w_sum += w;
// 				
// 				if(w > w_max)
// 					w_max = w;
// 				
// 				if(c > c_max)
// 					c_max = c;
// 				
// 				if(c < c_min)
// 					c_min = c;
				
				if(i%segs==0) printf("\n");
                                ROS_DEBUG("%f %f %f %d | ", c, m_particlelist[k].c, w, pxs[i]);
// 				m_particlelist[k].c = c;
// 				m_particlelist[k].w = w;
			}
			k++;
		}
// 		printf("\n %f %f %f %f\n", w_sum, w_max, c_max, c_min);
	
	}
	shadeCompare->unbind();
	glColorMask(1,1,1,1); glDepthMask(1);
	
// 	// normalize weights
// 	normalizeW();
// 
// 	// sort particles by likelihood and average most likely particles
// 	std::sort(m_particlelist.begin(), m_particlelist.end(), sortfunction);
// 	
// 	// calculate mean of distribution
// 	calcMean();
	
	cam_persp->Activate();
	glViewport(0,0,ip->getWidth(), ip->getHeight());
}



