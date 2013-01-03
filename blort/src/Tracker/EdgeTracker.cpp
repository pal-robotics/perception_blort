#include <ros/console.h>
#include <blort/Tracker/EdgeTracker.h>

using namespace Tracking;
using namespace TomGine;

// *** PRIVATE ***

// Process camera image (gauss, sobel, thinning, spreading, rendering)
void EdgeTracker::image_processing(unsigned char* image, GLenum format){
	
	// Load camera image to texture
	m_tex_frame->load(image, params.camPar.width, params.camPar.height, format);
	
	// Preprocessing for camera image
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glDepthMask(0);
	glColor3f(1,1,1);
	m_ip->flipUpsideDown(m_tex_frame, m_tex_frame);
	m_ip->gauss(m_tex_frame, m_tex_frame_ip[0]);
	m_ip->sobel(m_tex_frame_ip[0], m_tex_frame_ip[0], params.image_sobel_th);
	glDepthMask(1);
}

// Process camera image (gauss, sobel, thinning, spreading, rendering)
void EdgeTracker::image_processing(unsigned char* image, const TomGine::tgModel &m, const tgPose &p, GLenum format){
	
	// Load camera image to texture
	m_tex_frame->load(image, params.camPar.width, params.camPar.height, format);
	
	// Preprocessing for camera image
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glDepthMask(0);
	glColor3f(1,1,1);
	m_ip->flipUpsideDown(m_tex_frame, m_tex_frame);

	// Draw model (i.e. a virutal occluder)
	glDepthMask(1);
	glEnable(GL_DEPTH_TEST);
	drawModel(m,p);
	glDisable(GL_DEPTH_TEST);
	glDepthMask(0);
	m_tex_frame->copyTexImage2D(params.camPar.width, params.camPar.height);

	m_ip->gauss(m_tex_frame, m_tex_frame_ip[0]);
	m_ip->sobel(m_tex_frame_ip[0], m_tex_frame_ip[0], 0.03f);
	glDepthMask(1);
}

// particle filtering
void EdgeTracker::particle_filtering(ModelEntry* modelEntry){
	
	m_cam_perspective.Activate();
	
	// Calculate Zoom Direction and pass it to the particle filter
	TomGine::tgVector3 vCam = m_cam_perspective.GetPos();
	TomGine::tgVector3 vObj = TomGine::tgVector3(modelEntry->pose.t.x, modelEntry->pose.t.y, modelEntry->pose.t.z);
	modelEntry->vCam2Model = vObj - vCam;
	modelEntry->predictor->setCamViewVector(modelEntry->vCam2Model);
	
	
	glLineWidth(5);
	glColor3f(0.0f,0.0f,0.0f);
	m_tex_frame_ip[0]->bind();	// bind camera image
	float ns = 0.5f;
	
	for(int i=0; i<params.num_recursions; i++){
		if(params.num_recursions > 1)
			ns = 1.0f - float(i)/(params.num_recursions-1);
		
		glColor3f(0.0f,1.0f-ns,1.0f-ns);
		
		modelEntry->distribution.updateLikelihood(modelEntry->model, m_shadeEdgeCompare, false, params.convergence, m_showparticles);
		
		modelEntry->predictor->resample(modelEntry->distribution, modelEntry->num_particles, params.variation);
	}
	modelEntry->pose_prev = modelEntry->pose;
	modelEntry->pose = modelEntry->distribution.getMean();
	modelEntry->confidence_edge = modelEntry->pose.c;
}


// *** PUBLIC ***

EdgeTracker::EdgeTracker(){
	m_lock = false;
	m_showparticles = false;
	m_showmodel = true;
	m_draw_edges = false;
	m_tracker_initialized = false;
}

// Initialise function (must be called before tracking)
bool EdgeTracker::initInternal(){	
	
	int id;
	// Shader
	if((id = g_Resources->AddShader("edgetest", "edgetest.vert", "edgetest.frag")) == -1)
		exit(1);
	m_shadeEdgeCompare = g_Resources->GetShader(id);
	
	m_shadeEdgeCompare->bind();
	m_shadeEdgeCompare->setUniform("texFrame", 0);
	m_shadeEdgeCompare->setUniform("width", params.camPar.width);
	m_shadeEdgeCompare->setUniform("height", params.camPar.height);
	m_shadeEdgeCompare->unbind();
	
	return true;
}

bool EdgeTracker::track(){
	
	if(!m_tracker_initialized){
                ROS_DEBUG("[EdgeTracker::track()] Error tracker not initialised!");
		return false;
	}
	
	drawImage(0);
	
	for(unsigned i=0; i<m_modellist.size(); i++){
		// Recursive particle filtering
		if(!m_lock){
			particle_filtering(m_modellist[i]);		
		}
	}
	return true;
}

bool EdgeTracker::track(int id){
	ModelEntryList::iterator it = m_modellist.begin();
	while(it != m_modellist.end()){
		if(id==(*it)->id){
			if(!m_lock){
				particle_filtering(m_modellist[id]);		
			}
			return true;
		}
		++it;
	}
	return false;
}

// Draw result of edge tracking (particle with maximum likelihood)
void EdgeTracker::drawResult(float linewidth){
// 	float var = 0.0;
// 	var = m_distribution->getVariance(params.number_of_particles);
// 	glColor3f(1000*var,1-1000*var,0.0);
	m_cam_perspective.Activate();
	
	glColor3f(1.0,0.0,0.0);
	glLineWidth(linewidth);
	
// 		if(i==0){
// 			glDepthMask(0);
// 			if(m_draw_edges)
// 				m_ip->render(m_tex_frame_ip[params.m_spreadlvl]);
// 			else
// 				m_ip->render(m_tex_model_ip[params.m_spreadlvl]);
// 			glDepthMask(1);
// 		}
	
	for(unsigned i=0; i<m_modellist.size(); i++){
	
		m_modellist[i]->pose.Activate();
		
			glColorMask(0,0,0,0); glDepthMask(1);
			glClear(GL_DEPTH_BUFFER_BIT);
			m_modellist[i]->model.drawFaces();
			glColorMask(1,1,1,1);
			
			switch(m_showmodel){
				case 0:
					m_tex_frame_ip[0]->bind();
					m_shadeEdgeCompare->bind();
					m_shadeEdgeCompare->setUniform("analyze", true);
					m_modellist[i]->model.drawEdges();
					m_shadeEdgeCompare->unbind();
					break;
				case 1:
					m_modellist[i]->model.drawEdges();
					break;
				default:
					m_showmodel = 0;
					break;
			}
			//m_opengl.RenderSettings(true, true);
			
			
			glColor3f(1.0,1.0,1.0);
			
		m_modellist[i]->pose.Deactivate();
	}
	
}


