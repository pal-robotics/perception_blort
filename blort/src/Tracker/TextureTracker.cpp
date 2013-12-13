#include <ros/console.h>
#include <blort/Tracker/TextureTracker.h>
#include <blort/TomGine/tgLighting.h>
#include <blort/TomGine/tgMaterial.h>
#include <blort/TomGine/tgError.h>


using namespace std;
using namespace Tracking;
using namespace TomGine;

// *** PRIVATE ***

// Draw TrackerModel to screen, extract modelview matrix, perform image processing for model
void TextureTracker::model_processing(ModelEntry* modelEntry){
	
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	
	// Render camera image as background
	if(modelEntry->model.getTextured()){
		glDisable(GL_DEPTH_TEST);
		glDepthMask(0);
			m_ip->render(m_tex_frame);
		glDepthMask(1);
		glEnable(GL_DEPTH_TEST);
	}
	
	// Render textured model to screen
	m_cam_perspective.Activate();
	m_lighting.Activate();
	modelEntry->pose.Activate();
		modelEntry->model.restoreTexture();
		modelEntry->model.drawPass();
	
		// Extract modelview-projection matrix
		mat4 modelview, projection;
		glGetFloatv(GL_MODELVIEW_MATRIX, modelview);
		glGetFloatv(GL_PROJECTION_MATRIX, projection);
		modelEntry->modelviewprojection = projection * modelview;
		
		// pass new modelview matrix to shader
		m_shadeCompare->bind();
		m_shadeCompare->setUniform("modelviewprojection", modelEntry->modelviewprojection, GL_FALSE); // send matrix to shader
		m_shadeCompare->unbind();
		m_shadeConfidenceMM->bind();
		m_shadeConfidenceMM->setUniform("modelviewprojection", modelEntry->modelviewprojection, GL_FALSE); // send matrix to shader
		m_shadeConfidenceMM->unbind();
	modelEntry->pose.Deactivate();
	m_lighting.Deactivate();
		
	// Copy model rendered into image to texture (=reprojection to model)
	m_tex_model->copyTexImage2D(params.camPar.width, params.camPar.height);
		
	// perform image processing with reprojected image
	glDisable(GL_DEPTH_TEST);
	glDepthMask(0);
	
	// Extract edges in various line widths
	if(modelEntry->model.getTextured()){
		m_ip->gauss(m_tex_model, m_tex_model_ip[0]);
		m_ip->sobel(m_tex_model_ip[0], m_tex_model_ip[0], params.model_sobel_th, true);
	}else{
		m_ip->sobel(m_tex_model, m_tex_model_ip[0], params.model_sobel_th, true);
	}

        if(modelEntry->mask_geometry_edges){
		Texture mask;
		this->computeModelEdgeMask(modelEntry, mask);
		m_ip->thinning(m_tex_model_ip[0], m_tex_model_ip[0], &mask);
	}else{
		m_ip->thinning(m_tex_model_ip[0], m_tex_model_ip[0]);
	}

        for(unsigned i=1; i<params.num_spreadings; i++)
		m_ip->spreading(m_tex_model_ip[i-1], m_tex_model_ip[i]);

	glDepthMask(1);
}

// Particle filtering
void TextureTracker::particle_filtering(ModelEntry* modelEntry){
	int num_particles;
	m_cam_perspective.Activate();
	
	// Calculate Zoom Direction and pass it to the particle filter
	TomGine::tgVector3 vCam = m_cam_perspective.GetPos();
	TomGine::tgVector3 vObj = TomGine::tgVector3(modelEntry->pose.t.x, modelEntry->pose.t.y, modelEntry->pose.t.z);
	modelEntry->vCam2Model = vObj - vCam;
	modelEntry->predictor->setCamViewVector(modelEntry->vCam2Model);
	
	Particle variation = params.variation;
	
	for(unsigned i=0; i<modelEntry->num_recursions; i++){
// 		unsigned i=0;
		// TODO Evaluate if this makes sense (robustness, accuracy)
// 		float c_max = modelEntry->distribution.getMaxC();
// 		params.m_spreadlvl = (int)floor((params.num_spreadings)*(0.5-c_max));
// 		if(params.m_spreadlvl>=params.num_spreadings)
// 			params.m_spreadlvl = params.num_spreadings-1;
// 		else if(params.m_spreadlvl<0)
// 			params.m_spreadlvl = 0;

		// TODO Evaluate if this makes sense (robustness, accuracy)
		int j = (int)params.num_spreadings - (int)i;
		if(j > 1){
			params.m_spreadlvl = j - 1;
		}else{
			params.m_spreadlvl = 1;
		}
		float varred_t = 1.0f;
		float varred_q = 1.0f;
		float varred_z = 1.0f;
		if(modelEntry->num_recursions > 1){
			varred_t = 1.0f - 0.7f * float(i)/(modelEntry->num_recursions - 1);
			varred_q = 1.0f - 0.8f * float(i)/(modelEntry->num_recursions - 1);
			varred_z = 1.0f - 0.8f * float(i)/(modelEntry->num_recursions - 1);
		}
		variation.t = params.variation.t * varred_t;
		variation.q = params.variation.q * varred_q;
		variation.z = params.variation.z * varred_z;
		
// 		variation.z = params.variation.z * varred;
// 		ROS_DEBUG("%d %f %f %f\n", params.m_spreadlvl, variation.t.x, variation.r.x*180/PI, varred_t);
		
// 		if(i == modelEntry->num_recursions - 1)
// 			m_showparticles = true;
// 		else
// 			m_showparticles = false;
		
                //BENCE: commented these, they are not really used
//                m_tex_model_ip[params.m_spreadlvl]->bind(0);
//                m_tex_frame_ip[params.m_spreadlvl]->bind(1);
//                m_tex_model->bind(2);
//                m_tex_frame->bind(3);

		// TODO Evaluate if this makes sense (robustness, accuracy)
// 		params.kernel_size = (int)floor(0.5*(params.max_kernel_size)*(1.0-c_max));
// 		m_shadeCompare->bind();
// 		m_shadeCompare->setUniform("kernelsize", params.kernel_size);
// 		m_shadeCompare->unbind();
		
		// update importance weights and confidence levels
// 		ROS_DEBUG("Recursion[%d] %d c:%f p: ", i, params.m_spreadlvl, modelEntry->distribution.getParticle(0).c);
// 		modelEntry->distribution.getParticle(0).Print();
		modelEntry->distribution.updateLikelihood(modelEntry->model, m_shadeCompare, 1, params.convergence, m_showparticles);
		
		//modelEntry->distribution = d1;
// 		ROS_DEBUG("%f\n", ( 1.0 - i * 0.5 / (float)(modelEntry->num_recursions-1)));
		// TODO Evaluate if this makes sense (accuracy)
// 		p = (params.variation * ( 1.0 - i * 0.8 / (float)(modelEntry->num_recursions-1)));
		
		// "Attention mechanism" modifies number of particles indirect proportional to the overall confidence
		//num_particles = modelEntry->num_particles * (1.0 - c_max);
		num_particles = modelEntry->num_particles;
		
		// predict movement of object

		modelEntry->predictor->resample(modelEntry->distribution, num_particles, variation, (i==0));

		// set timestep to 0.0 for further recursion (within same image)
		modelEntry->predictor->updateTime(0.0);
	}
	tgCheckError("TextureTracker::particle_filtering");
	// weighted mean
	modelEntry->pose_prev = modelEntry->pose;
	modelEntry->pose = modelEntry->distribution.getMean();
// 	modelEntry->pose = modelEntry->distribution.getParticle(0);
}



// *** PUBLIC ***

TextureTracker::TextureTracker(){
	m_lock = false;
	m_showparticles = false;
	m_showmodel = 0;
	m_draw_edges = false;
	m_tracker_initialized = false;
	m_drawimage = false;
}

TextureTracker::~TextureTracker(){
	delete(m_tex_model);
	for(unsigned i=0; i<params.num_spreadings; i++){
		delete(m_tex_model_ip[i]);
	}
}

// Initialise function (must be called before tracking)
bool TextureTracker::initInternal(){	
	
	int id;

	// Shader
	if((id = g_Resources->AddShader("texEdgeTest", "texEdgeTest.vert", "texEdgeTest.frag")) == -1)
		exit(1);
	m_shadeTexEdgeTest = g_Resources->GetShader(id);
	
	if((id = g_Resources->AddShader("texColorTest", "texColorTest.vert", "texColorTest.frag")) == -1)
		exit(1);
	m_shadeTexColorTest = g_Resources->GetShader(id);
	
	if((id = g_Resources->AddShader("mmConfidence", "mmConfidence.vert", "mmConfidence.frag")) == -1)
		exit(1);
	m_shadeConfidenceMM = g_Resources->GetShader(id);
	
	m_shadeCompare = m_shadeTexEdgeTest;
	
	// Texture
	m_tex_model = new Texture();
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
	
	m_tex_model_ip.push_back( new Texture() );
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
	for(int i=0; i<(int)params.num_spreadings-1; i++){
		m_tex_model_ip.push_back( new Texture() );
		glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
	}
	
	// Load 
	float w = (float)params.camPar.width;
	float h = (float)params.camPar.height;
	
	GLfloat offX[9] = { -1.0f/w, 0.0f, 1.0f/w,
						-1.0f/w, 0.0f, 1.0f/w,
						-1.0f/w, 0.0f, 1.0f/w };
	GLfloat offY[9] = {  1.0f/h, 1.0f/h,  1.0f/h,
						 0.0f,   0.0f,    0.0f,
						-1.0f/h,-1.0f/h, -1.0f/h };
                        
	m_shadeTexEdgeTest->bind();
	m_shadeTexEdgeTest->setUniform("tex_model_edge", 0);
	m_shadeTexEdgeTest->setUniform("tex_frame_edge", 1);
	m_shadeTexEdgeTest->setUniform("fTol", params.edge_tolerance);
	m_shadeTexEdgeTest->setUniform( "mOffsetX", mat3(offX), GL_FALSE );
	m_shadeTexEdgeTest->setUniform( "mOffsetY", mat3(offY), GL_FALSE );
	m_shadeTexEdgeTest->setUniform("drawcolor", vec4(1.0f,0.0f,0.0f,0.0f));
	m_shadeTexEdgeTest->setUniform("kernelsize", (GLint)params.kernel_size);
	m_shadeTexEdgeTest->unbind();
	
	m_shadeTexColorTest->bind();
	m_shadeTexColorTest->setUniform("tex_model_color", 2);
	m_shadeTexColorTest->setUniform("tex_frame_color", 3);
	m_shadeTexColorTest->setUniform("fTol", params.edge_tolerance);
	m_shadeTexColorTest->setUniform( "mOffsetX", mat3(offX), GL_FALSE );
	m_shadeTexColorTest->setUniform( "mOffsetY", mat3(offY), GL_FALSE );
	m_shadeTexColorTest->setUniform("drawcolor", vec4(0.0f,0.0f,1.0f,0.0f));
	m_shadeTexColorTest->setUniform("kernelsize", (GLint)params.kernel_size);
	m_shadeTexColorTest->unbind();
	
	m_shadeConfidenceMM->bind();
	m_shadeConfidenceMM->setUniform("tex_model_color", 2);
	m_shadeConfidenceMM->setUniform("tex_frame_color", 3);
	m_shadeConfidenceMM->setUniform("fTol", params.edge_tolerance);
	m_shadeConfidenceMM->setUniform( "mOffsetX", mat3(offX), GL_FALSE );
	m_shadeConfidenceMM->setUniform( "mOffsetY", mat3(offY), GL_FALSE );
	m_shadeConfidenceMM->setUniform("drawcolor", vec4(0.0f,0.0f,1.0f,0.0f));
	m_shadeConfidenceMM->setUniform("kernelsize", (GLint)params.kernel_size);
	m_shadeConfidenceMM->unbind();
	
	tgLight light;
	tgVector3 cam_f = m_cam_perspective.GetF();
	light.ambient = vec4(0.4f,0.4f,0.4f,1.0f);
	light.diffuse = vec4(1.0f,1.0f,1.0f,1.0f);
	light.specular = vec4(1.0f,1.0f,1.0f,1.0f);
	light.position = vec4(-cam_f.x, -cam_f.y, -cam_f.z, 1.0f);
	m_lighting.ApplyLight(light,0);
	
	return true;
}

float TextureTracker::evaluateParticle(ModelEntry* modelEntry)
{
	return evaluateParticle(modelEntry, m_shadeCompare);
}

// evaluate single particle
float TextureTracker::evaluateParticle(ModelEntry* modelEntry, Shader* shader){
	unsigned int queryMatches;
	unsigned int queryEdges;
	int v, d;
	float c;
	
	glGenQueriesARB(1, &queryMatches);
	glGenQueriesARB(1, &queryEdges);

	m_cam_perspective.Activate();
	//glViewport(0,0,256,256);
	glColorMask(0,0,0,0); glDepthMask(0);
	glColor3f(1.0,1.0,1.0);

	m_tex_model_ip[params.m_spreadlvl]->bind(0);
	m_tex_frame_ip[params.m_spreadlvl]->bind(1);	
	m_tex_model->bind(2);
	m_tex_frame->bind(3);
	
	// Draw particles and count pixels
	shader->bind();
	shader->setUniform("analyze", false);
    
	modelEntry->pose.Activate();
	
		// Draw all model edge pixels
		glBeginQueryARB(GL_SAMPLES_PASSED_ARB, queryEdges);
		shader->setUniform("compare", false);
		if(m_showparticles)
			glColorMask(1,1,1,1);
		modelEntry->model.drawTexturedFaces();
		modelEntry->model.drawUntexturedFaces();
		glEndQueryARB(GL_SAMPLES_PASSED_ARB);
		
		glColorMask(0,0,0,0);
		
		// Draw matching model edge pixels using a shader
		glBeginQueryARB(GL_SAMPLES_PASSED_ARB, queryMatches);
		shader->setUniform("compare", true);
		shader->setUniform("textured", true);
		modelEntry->model.drawTexturedFaces();
		shader->setUniform("textured", false);
		modelEntry->model.drawUntexturedFaces();
		glEndQueryARB(GL_SAMPLES_PASSED_ARB);
	
	modelEntry->pose.Deactivate();

	shader->unbind();

	glGetQueryObjectivARB(queryMatches, GL_QUERY_RESULT_ARB, &d);
	glGetQueryObjectivARB(queryEdges, GL_QUERY_RESULT_ARB, &v);
	glColorMask(1,1,1,1); glDepthMask(1);
	glDeleteQueriesARB(1, &queryMatches);
	glDeleteQueriesARB(1, &queryEdges);
	
	c = modelEntry->distribution.confidenceFunction(d,v);
	

	
	return c;
}

// Process camera image (gauss, sobel, thinning, spreading, rendering)
void TextureTracker::image_processing(unsigned char* image, GLenum format){
	
	// Load camera image to texture
	m_tex_frame->load(image, params.camPar.width, params.camPar.height, format);
	
	// Preprocessing for camera image
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glDisable(GL_DEPTH_TEST);
	glDepthMask(0);
	glColor3f(1.0,1.0,1.0);
	
	m_ip->flipUpsideDown(m_tex_frame, m_tex_frame);
	
	m_ip->gauss(m_tex_frame, m_tex_frame_ip[0]);
        m_ip->sobel(m_tex_frame_ip[0], m_tex_frame_ip[0], params.image_sobel_th, true);
        m_ip->thinning(m_tex_frame_ip[0], m_tex_frame_ip[0]);
        for(unsigned i=1; i<params.num_spreadings; i++)
                m_ip->spreading(m_tex_frame_ip[i-1], m_tex_frame_ip[i]);
	
	glDepthMask(1);
}

// Process camera image (gauss, sobel, thinning, spreading, rendering)
void TextureTracker::image_processing(unsigned char* image, const TomGine::tgModel &m, const tgPose &p, GLenum format){
	
	// Load camera image to texture
	m_tex_frame->load(image, params.camPar.width, params.camPar.height, format);
	
	// Preprocessing for camera image
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glDisable(GL_DEPTH_TEST);
	glDepthMask(0);
	glColor3f(1.0,1.0,1.0);
	
	m_ip->flipUpsideDown(m_tex_frame, m_tex_frame);

	// Draw model (i.e. a virutal occluder)
	glDepthMask(1);
	glEnable(GL_DEPTH_TEST);
	drawModel(m,p);
	glDisable(GL_DEPTH_TEST);
	glDepthMask(0);
	m_tex_frame->copyTexImage2D(params.camPar.width, params.camPar.height);

	m_ip->gauss(m_tex_frame, m_tex_frame_ip[0]);
	m_ip->sobel(m_tex_frame_ip[0], m_tex_frame_ip[0], params.image_sobel_th, true);
        m_ip->thinning(m_tex_frame_ip[0], m_tex_frame_ip[0]);
        for(unsigned i=1; i<params.num_spreadings; i++)
                m_ip->spreading(m_tex_frame_ip[i-1], m_tex_frame_ip[i]);
	
	glDepthMask(1);
}

void TextureTracker::image_processing(unsigned char* image, int model_id, const tgPose &p, GLenum format){
	
	// Load camera image to texture
	m_tex_frame->load(image, params.camPar.width, params.camPar.height, format);
	
	// Preprocessing for camera image
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glDisable(GL_DEPTH_TEST);
	glDepthMask(0);
	glColor3f(1.0,1.0,1.0);
	
	m_ip->flipUpsideDown(m_tex_frame, m_tex_frame);
	
// 	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// Draw model (i.e. a virutal occluder)
	ModelEntryList::iterator it = m_modellist.begin();
	while(it != m_modellist.end()){
		if(model_id==(*it)->id){
			m_cam_perspective.Activate();
			glClear(GL_DEPTH_BUFFER_BIT);
			m_lighting.Activate();
			glEnable(GL_DEPTH_TEST);
			glDepthMask(1);
			p.Activate();
			
			//(*it)->model.restoreTexture();
			(*it)->model.drawPass();
			
			p.Deactivate();
			glDepthMask(0);
			glDisable(GL_DEPTH_TEST);
			m_lighting.Deactivate();
			glClear(GL_DEPTH_BUFFER_BIT);
		}
		++it;
	}
	m_tex_frame->copyTexImage2D(params.camPar.width, params.camPar.height);

	m_ip->gauss(m_tex_frame, m_tex_frame_ip[0]);
	m_ip->sobel(m_tex_frame_ip[0], m_tex_frame_ip[0], params.image_sobel_th, true);
	m_ip->thinning(m_tex_frame_ip[0], m_tex_frame_ip[0]);
	for(unsigned i=1; i<params.num_spreadings; i++)
		m_ip->spreading(m_tex_frame_ip[i-1], m_tex_frame_ip[i]);
	
	glDepthMask(1);
}

bool TextureTracker::track()
{
  std::vector<bool> tracking_objects(m_modellist.size(), true);
  return track(tracking_objects);
}

bool TextureTracker::track(const std::vector<bool> & tracking_objects)
{
  if(!m_tracker_initialized)
  {
    ROS_ERROR("[TextureTracker::track()] Error tracker not initialised!\n");
    return false;
  }

  // Track models
  for(unsigned i=0; i<m_modellist.size(); i++)
  {
      if(tracking_objects[i])
      {
        track(m_modellist[i]);
      }
  }

  tgCheckError("TextureTracker::track()");
  return true;
}

bool TextureTracker::track(ModelEntry *modelEntry)
{

	// Process model (texture reprojection, edge detection)
  model_processing(modelEntry);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	if(m_drawimage)
		drawImage(NULL);

	// Apply particle filtering
	if(!modelEntry->lock){
		particle_filtering(modelEntry);
		if(!m_cam_perspective.GetFrustum()->PointInFrustum(modelEntry->pose.t.x, modelEntry->pose.t.y, modelEntry->pose.t.z))
			reset();
	}else{
		modelEntry->pose.c = evaluateParticle(modelEntry, m_shadeCompare);
	}

	modelEntry->confidence_edge = evaluateParticle(modelEntry, m_shadeTexEdgeTest);
	modelEntry->confidence_color = evaluateParticle(modelEntry, m_shadeTexColorTest);    
	
	modelEntry->filter_pose();
  modelEntry->evaluate_states(params.variation, params.num_recursions,
                              params.c_th_base, params.c_th_min, params.c_th_fair,
                              params.c_mv_not, params.c_mv_slow, params.c_th_lost);

  m_ftime = (float)m_timer.Update();
	return true;
}

bool TextureTracker::track(int id)
{
  if(!m_tracker_initialized)
  {
    ROS_ERROR("[TextureTracker::track(int)] Error tracker not initialised!\n");
		return false;
	}
	
	ModelEntryList::iterator it = m_modellist.begin();
  while(it != m_modellist.end())
  {
    if(id==(*it)->id)
    {
			track(m_modellist[id]);
			tgCheckError("TextureTracker::track(int)");
			return true;
		}
		++it;
	}
	
	return false;
}

// grabs texture from camera image and attaches it to faces of model
void TextureTracker::textureFromImage(bool use_num_pixels){
	std::vector<tgVertex> vertices;
	
	Parameter tmpParams = params;
	
	params.num_particles = 500;
	params.num_recursions = 10;
	params.m_spreadlvl = 0;
	params.variation = params.variation * 0.1;
	setKernelSize(0);

	track();

	params = tmpParams;
	setKernelSize(params.kernel_size);
	
	for(unsigned i=0; i<m_modellist.size(); i++){

		m_cam_perspective.Activate();
		vector<unsigned> faceUpdateList = m_modellist[i]->model.getFaceUpdateList(m_modellist[i]->pose, 
					vec3(m_modellist[i]->vCam2Model.x, m_modellist[i]->vCam2Model.y, m_modellist[i]->vCam2Model.z),
					params.minTexGrabAngle,
					use_num_pixels);

		if(!faceUpdateList.empty()){
			vertices.clear();
			m_modellist[i]->model.textureFromImage(	m_tex_frame,
                                                                params.camPar.width, params.camPar.height,
                                                                m_modellist[i]->pose,
                                                                vec3(m_modellist[i]->vCam2Model.x, m_modellist[i]->vCam2Model.y, m_modellist[i]->vCam2Model.z),
                                                                params.minTexGrabAngle,
                                                                faceUpdateList,
                                                                vertices,
                                                                &m_cam_perspective);
		}
		faceUpdateList.clear();
	}
}

// grabs texture from camera image and attaches it to faces of model
void TextureTracker::textureFromImage(int id, const TomGine::tgPose &pose, bool use_num_pixels){
	std::vector<tgVertex> vertices;
	
        ModelEntry* modelEntry = 0;
	ModelEntryList::iterator it = m_modellist.begin();
	while(it != m_modellist.end()){
		if(id==(*it)->id){
			modelEntry = (*it);
		}
		++it;
	}
	
	modelEntry->pose = pose;
	
	m_cam_perspective.Activate();
	vector<unsigned> faceUpdateList = modelEntry->model.getFaceUpdateList(modelEntry->pose, 
				vec3(modelEntry->vCam2Model.x, modelEntry->vCam2Model.y, modelEntry->vCam2Model.z),
				params.minTexGrabAngle,
				use_num_pixels);

	if(!faceUpdateList.empty()){
		vertices.clear();

                modelEntry->model.textureFromImage(m_tex_frame,
                                                   params.camPar.width, params.camPar.height,
                                                   modelEntry->pose,
                                                   vec3(modelEntry->vCam2Model.x, modelEntry->vCam2Model.y, modelEntry->vCam2Model.z),
                                                   params.minTexGrabAngle,
                                                   faceUpdateList,
                                                   vertices,
                                                   &m_cam_perspective);
	}
	faceUpdateList.clear();
}

void TextureTracker::untextureModels(){
	for(unsigned i=0; i<m_modellist.size(); i++){
		m_modellist[i]->model.releasePassList();	
	}
}

// Draw result of texture tracking (particle with maximum likelihood)
void TextureTracker::drawResult(float linewidth){
		
	m_cam_perspective.Activate();
	m_lighting.Activate();

	glEnable(GL_DEPTH_TEST);
	glClear(GL_DEPTH_BUFFER_BIT);
	
	for(unsigned i=0; i<m_modellist.size(); i++){
        if(m_modellist[i]->st_quality != Tracking::ST_LOST)
        {
            drawModelEntry(m_modellist[i], linewidth);
        }
	}
	
// 	for(int i=0; i<m_hypotheses.size(); i++){
// 		drawModelEntry(m_hypotheses[i]);
// 	}
	
	m_lighting.Deactivate();
	tgCheckError("TextureTracker::drawResult B");
}

void TextureTracker::drawModelEntry(ModelEntry* modelEntry, float linewidth){

		modelEntry->pose.Activate();
		
		switch(m_showmodel){
			case 0:
				modelEntry->model.restoreTexture();
				modelEntry->model.drawPass(false);
				break;
			case 1:
				m_lighting.Deactivate();
				//glColorMask(0,0,0,0);
				//modelEntry->model.drawFaces();
				//glColorMask(1,1,1,1);
				if(linewidth<1.0f)
					linewidth = 1.0f;
				glLineWidth(linewidth);
				modelEntry->model.drawEdges();
				glColor3f(1.0,1.0,1.0);
				break;
			case 2:
				m_tex_model_ip[params.m_spreadlvl]->bind(0);
				m_tex_frame_ip[params.m_spreadlvl]->bind(1);	
				m_tex_model->bind(2);
				m_tex_frame->bind(3);
				m_shadeCompare->bind();
				m_shadeCompare->setUniform("analyze", true);
				m_shadeCompare->setUniform("compare", true);
				m_shadeCompare->setUniform("textured", true);
				modelEntry->model.drawTexturedFaces();
				m_shadeCompare->setUniform("textured", false);
				modelEntry->model.drawUntexturedFaces();
				m_shadeCompare->unbind();
				break;
			case 3:
				break;
			default:
				m_showmodel = 0;
				break;			
		}
				
// 		m_modellist[i]->model.drawCoordinates();
// 		m_modellist[i]->model.drawNormals();
		modelEntry->pose.Deactivate();
}

void TextureTracker::drawTrackerModel(int id, const TomGine::tgPose &p, float linewidth)
{
	m_cam_perspective.Activate();
	ModelEntryList::iterator it = m_modellist.begin();
	while(it != m_modellist.end()){
		if(id==(*it)->id){
			ModelEntry* modelEntry = (*it);
			p.Activate();
				switch(m_showmodel){
					case 0:
						modelEntry->model.restoreTexture();
						modelEntry->model.drawPass(false);
						break;
					case 1:
						m_lighting.Deactivate();
						//glColorMask(0,0,0,0);
						//modelEntry->model.drawFaces();
						//glColorMask(1,1,1,1);
						if(linewidth<1.0f)
							linewidth = 1.0f;
						glLineWidth(linewidth);
						modelEntry->model.drawEdges();
						glColor3f(1.0,1.0,1.0);
						break;
					case 2:
					case 3:
						break;
					default:
						m_showmodel = 0;
						break;			
				}
			p.Deactivate();
		}
		++it;
	}
}

void TextureTracker::evaluatePDF(int id,
                                 float x_min, float y_min,
                                 float x_max, float y_max,
                                 int res,
                                 const char* meshfile, const char* xfile)
{
	ModelEntryList::iterator it = m_modellist.begin();
	while(it != m_modellist.end()){
		if(id==(*it)->id){
			vector<float> pdf;
			pdf = getPDFxy((*it), x_min, y_min, x_max, y_max, res);
			savePDF(pdf, x_min, y_min, x_max, y_max, res, meshfile, xfile);			
			return;
		}
		++it;
	}
}

// Plots pdf in x-y plane (with z and rotational DOF locked)
vector<float> TextureTracker::getPDFxy(ModelEntry* modelEntry,
                                       float x_min, float y_min,
                                       float x_max, float y_max,
                                       int res)
{
	int i = 0;
  ROS_DEBUG("Evaluating PDF constrained to x,y movement only");
	float x_step = (x_max-x_min) / res;
	float y_step = (y_max-y_min) / res;
	float scale = 0.1f;
	
	x_min = (modelEntry->pose.t.x += x_min);
	y_min = (modelEntry->pose.t.y += y_min);
	
	vector<float> vPDF;
	vPDF.assign(res*res, 0.0);
	
	TomGine::tgFrustum* frustum = m_cam_perspective.GetFrustum();
	
	
	float p;
	i=0;
	modelEntry->pose.t.y = y_min;
	for(int n=0; n<res; n++){
		modelEntry->pose.t.x = x_min;
		for(int m=0; m<res; m++){
			if(frustum->PointInFrustum(modelEntry->pose.t.x, modelEntry->pose.t.y, modelEntry->pose.t.z)){
				p = evaluateParticle(modelEntry, m_shadeTexEdgeTest) * scale;
			}else{
				p = 0.0;
			}
			vPDF[i] = p;
			
			i++;
			modelEntry->pose.t.x += x_step;
		}
		modelEntry->pose.t.y += y_step;
	}
	
//  	glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
// 	glEnable(GL_BLEND);
// 	glBegin(GL_QUADS);
// 		glColor4f(1.0,0.0,0.0,0.5);
// 		glVertex3f(x_min, y_min, 0.0);
// 		glVertex3f(x_min+x_step*res, y_min, 0.0);
// 		glVertex3f(x_min+x_step*res, y_min+y_step*res, 0.0);
// 		glVertex3f(x_min, y_min+y_step*res, 0.0);
// 	glEnd();
// 	glDisable(GL_BLEND);
// 	
// 	swap();
// 	usleep(3000000);
	
	return vPDF;
}

// draws a rectangular terrain using a heightmap
void TextureTracker::savePDF(vector<float> vPDFMap,
                             float x_min, float y_min,
                             float x_max, float y_max,
                             unsigned res,
                             const char* meshfile, const char* xfile)
{
	unsigned i,d,x,y;
	
	float x_step = (x_max-x_min) / res;
	float y_step = (y_max-y_min) / res;
	
	vector<vec3> vertexlist;
	vector<vec3> normallist;
	vector<vec2> texcoordlist;
	vector<unsigned int> indexlist;
	
	for(y=0; y<res; y++){
		for(x=0; x<res; x++){
			vec3 v[5], n;
			vec2 tc;
			d=0;
			
			v[0].x = x_min + float(x)*x_step;
			v[0].y = y_min + float(y)*y_step;
			v[0].z = vPDFMap[y*res+x];
			
			if(x<res-1){
			v[1].x = x_min + float(x+1)*x_step;
			v[1].y = y_min + float(y)*y_step;
			v[1].z = vPDFMap[y*res+(x+1)];
			n += (v[1]-v[0]);
			d++;
			}
			
			if(y<res-1){
			v[2].x = x_min + float(x)*x_step;
			v[2].y = y_min + float(y+1)*y_step;
			v[2].z = vPDFMap[(y+1)*res+x];
			n += (v[2]-v[0]);
			d++;
			}
			
			if(x>0){
			v[3].x = x_min + float(x-1)*x_step;
			v[3].y = y_min + float(y)*y_step;
			v[3].z = vPDFMap[y*res+(x-1)];
			n += (v[3]-v[0]);
			d++;
			}
			
			if(y>0){
			v[4].x = x_min + float(x)*x_step;
			v[4].y = y_min + float(y-1)*y_step;
			v[4].z = vPDFMap[(y-1)*res+x];
			n += (v[4]-v[0]);
			d++;
			}
			
			n = n * (1.0f/float(d));
			
			tc.x = float(x)/res;
			tc.y = float(y)/res;
			
			vertexlist.push_back(v[0]);
			normallist.push_back(n);
			texcoordlist.push_back(tc);
			
			if(x<res-1 && y<res-1){
				indexlist.push_back(y*res+x);
				indexlist.push_back(y*res+(x+1));
				indexlist.push_back((y+1)*res+(x+1));
				
				indexlist.push_back(y*res+x);
				indexlist.push_back((y+1)*res+(x+1));
				indexlist.push_back((y+1)*res+x);
			}
		}
	}
	/*
	glClear(GL_DEPTH_BUFFER_BIT);
	m_lighting.Activate();
	
	glEnableClientState(GL_VERTEX_ARRAY); glVertexPointer(3, GL_FLOAT, 0, &vertexlist[0]);
	glEnableClientState(GL_NORMAL_ARRAY); glNormalPointer(GL_FLOAT, 0, &normallist[0]);
	glEnableClientState(GL_TEXTURE_COORD_ARRAY); glTexCoordPointer(2, GL_FLOAT, 0, &texcoordlist[0]);
	
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	glDrawElements(GL_TRIANGLES, 6 * (xres-1) * (yres-1), GL_UNSIGNED_INT, &indexlist[0]);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	
	glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_NORMAL_ARRAY);
	glDisableClientState(GL_TEXTURE_COORD_ARRAY);
	
	m_lighting.Deactivate();
	*/
	if(meshfile){
		FILE* fd = fopen(meshfile,"w");
		fprintf(fd, "ply\nformat ascii 1.0\n");
		fprintf(fd, "element vertex %d\n", res*res);
		fprintf(fd, "property float x\n");
		fprintf(fd, "property float y\n");
		fprintf(fd, "property float z\n");
		fprintf(fd, "property float nx\n");
		fprintf(fd, "property float ny\n");
		fprintf(fd, "element face %d\n", (res-1)*(res-1)*2);
		fprintf(fd, "property list uchar uint vertex_indices\n");
		fprintf(fd, "end_header\n");
		
		for(i=0; i<vertexlist.size(); i++){
			vec3 v = vertexlist[i];
			vec3 n = normallist[i];
			fprintf(fd, "%f %f %f %f %f\n", v.x, v.y, v.z, n.x, n.y);
		}
		
		for(i=0; i<indexlist.size(); i+=3){
			fprintf(fd, "3 %d %d %d\n", indexlist[i], indexlist[i+1], indexlist[i+2]);
		}
		
    ROS_DEBUG("  output written to '%s'", meshfile);
		
		fclose(fd);
	}
	
	if(xfile){
		FILE* fd2 = fopen(xfile,"w");
		y = res>>1;
		for(x=0; x<res; x++){
			fprintf(fd2, "%f\n", vPDFMap[y*res+x]);
		}
    ROS_DEBUG("  output written to '%s'", xfile);
		fclose(fd2);
	}
}

// get m_tex_model
cv::Mat TextureTracker::getModelTexture()
{
    return m_tex_model->toCvMat();
}
