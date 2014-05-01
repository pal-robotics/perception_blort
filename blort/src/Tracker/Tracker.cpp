#include <ros/console.h>

#include <blort/Tracker/Tracker.h>
#include <stdexcept>
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>

using namespace Tracking;
using namespace TomGine;
using namespace std;

// *** PUBLIC ***

Tracker::Parameter::Parameter(){
    // Default parameter
    camPar.width = 640;
    camPar.height = 480;
    model_id_count = 0;
    hypotheses_id_count = 0;
    num_particles = 100;
    num_recursions = 2;
    hypotheses_trials = 5;
    convergence = 5;
    edge_tolerance = 45.0f * pi/180.0f;
    num_spreadings = 3;
    m_spreadlvl = 2;
    variation = Particle(vec3(0.01f, 0.01f, 0.01f), vec3(0.1f,0.1f,0.1f), 0.01f, 0.01f, 0.01f, TomGine::tgQuaternion());
    minTexGrabAngle = 3.0f*PI/4.0f;
    max_kernel_size = 3;
    kernel_size = 1;
    modelPath = string("../Resources/model/");
    texturePath = string("../Resources/texture/");
    shaderPath = string("../Resources/shader/");
    image_sobel_th = 0.02f;
    model_sobel_th = 0.03f;
    c_th_base=0.6f;
    c_th_min=0.3f;
    c_th_fair=0.5f;
    c_mv_not=0.01f;
    c_mv_slow=0.05f;
    c_th_lost=0.1f;
    pred_no_convergence=0.2f;
}

Tracker::Tracker(){
    // Default flags
    m_lock = false;
    m_showparticles = false;
    m_showmodel = 1;
    m_draw_edges = false;
    m_tracker_initialized = false;
}

Tracker::~Tracker(){
    unsigned i;
    delete(m_tex_frame);
    for(i=0; i<m_tex_frame_ip.size(); i++){
        delete(m_tex_frame_ip[i]);
    }
    for(i=0; i<m_modellist.size(); i++){
        delete(m_modellist[i]);
    }
    delete g_Resources;
}

bool Tracker::initGL(){
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    
    // commented because this gives a segfault at strstr(.......)
    //        const GLubyte *str;
    //        int glOcclusionQueryAvailable;
    //
    //        // Check for Extension
    //        str = glGetString(GL_EXTENSIONS);
    //
    //        glOcclusionQueryAvailable = (strstr((const char *)str, "GL_ARB_occlusion_query") != NULL);
    //        if(!glOcclusionQueryAvailable){
    //                char errmsg[256];
    //                sprintf(errmsg, "[OpenGLControl] Error OpenGL extension 'GL_ARB_occlusion_query' not available. Your graphic card does not support this extension or the hardware driver for your graphic card is not installed properly!");
    //                throw std::runtime_error(errmsg);
    //        }
    
    return true;
}

bool Tracker::loadCamParsFromINI(const char* camCalFile, const char* poseCalFile){
    CDataFile camCDF, poseCDF;
    
    if(!camCDF.Load(camCalFile)){
        char errmsg[128];
        sprintf(errmsg, "[Tracker::loadCamParsFromINI] Cannot open file %s", camCalFile);
        throw std::runtime_error(errmsg);
    }
    
    // Camera Parameters
    params.camPar.width = camCDF.GetInt("w");
    params.camPar.height = camCDF.GetInt("h");
    params.camPar.fx = camCDF.GetFloat("fx");
    params.camPar.fy = camCDF.GetFloat("fy");
    params.camPar.cx = camCDF.GetFloat("cx");
    params.camPar.cy = camCDF.GetFloat("cy");
    params.camPar.k1 = camCDF.GetFloat("k1");
    params.camPar.k2 = camCDF.GetFloat("k2");
    params.camPar.k3 = 0.0f;
    params.camPar.p1 = camCDF.GetFloat("p1");
    params.camPar.p2 = camCDF.GetFloat("p2");
    
    //printf("%d %d, %f %f, %f %f, %f %f %f, %f %f\n",
    //	params.camPar.width,
    //	params.camPar.height,
    //	params.camPar.fx,
    //	params.camPar.fy,
    //	params.camPar.cx,
    //	params.camPar.cy,
    //	params.camPar.k1,
    //	params.camPar.k2,
    //	params.camPar.k3,
    //	params.camPar.p1,
    //	params.camPar.p2);
    
    params.camPar.zFar = 4.0f;
    params.camPar.zNear = 0.1f;
    
    if(!poseCDF.Load(poseCalFile)){
        char errmsg[128];
        sprintf(errmsg, "[Tracker::loadCamParsFromINI] Cannot open file %s", poseCalFile);
        throw std::runtime_error(errmsg);
    }
    
    vec3 p, r;
    string pose = poseCDF.GetString("pose");
    sscanf( pose.c_str(), "[%f %f %f] [%f %f %f]",
            &(p.x), &(p.y), &(p.z), &(r.x), &(r.y), &(r.z) );
    
    //printf("%s\n", pose.c_str());
    //printf("%f %f %f, %f %f %f\n", p.x, p.y, p.z, r.x, r.y, r.z);
    
    params.camPar.pos.x = p.x;
    params.camPar.pos.y = p.y;
    params.camPar.pos.z = p.z;
    params.camPar.rot.fromRotVector(r);
    return true;
}

// load INI file of tracker
bool Tracker::loadTrackParsFromINI(const char* inifile){
    CDataFile cdfParams;
    
    if(!cdfParams.Load(inifile)){
        char errmsg[128];
        sprintf(errmsg, "[Tracker::loadTrackParsFromINI] Cannot open file %s", inifile);
        throw std::runtime_error(errmsg);
    }
    
    float dpi = pi/180.0f;
    
    // Constraints
    params.variation.r.x = cdfParams.GetFloat("r.x", "Constraints") * dpi;
    params.variation.r.y = cdfParams.GetFloat("r.y", "Constraints") * dpi;
    params.variation.r.z = cdfParams.GetFloat("r.z", "Constraints") * dpi;
    params.variation.t.x 	= cdfParams.GetFloat("t.x", "Constraints");
    params.variation.t.y 	= cdfParams.GetFloat("t.y", "Constraints");
    params.variation.t.z 	= cdfParams.GetFloat("t.z", "Constraints");
    params.variation.z 		= cdfParams.GetFloat("z", "Constraints");
    
    // Performance
    params.num_recursions = cdfParams.GetInt("recursions", "Performance");
    params.num_particles = cdfParams.GetInt("particles", "Performance");
    params.hypotheses_trials = cdfParams.GetInt("hypotheses", "Performance");
    params.convergence = cdfParams.GetInt("convergence", "Performance");
    
    // Resource Path
    params.modelPath = cdfParams.GetString("ModelPath", "ResourcePath");
    params.texturePath = cdfParams.GetString("TexturePath", "ResourcePath");
    params.shaderPath = cdfParams.GetString("ShaderPath", "ResourcePath");
    
    // Other
    params.edge_tolerance = cdfParams.GetFloat("EdgeMatchingTolerance", "Other") * dpi;
    params.minTexGrabAngle = cdfParams.GetFloat("MinTextureGrabAngle", "Other") * dpi;
    params.num_spreadings =  cdfParams.GetInt("NumberOfSpreadings", "Other");
    params.max_kernel_size = cdfParams.GetInt("MaxKernelSize", "Other");
    
    params.model_sobel_th = cdfParams.GetFloat("ModelSobelThreshold", "Other");
    params.image_sobel_th = cdfParams.GetFloat("ImageSobelThreshold", "Other");
    params.pred_no_convergence = cdfParams.GetFloat("PredictorNoConvergence", "Other");
    
    params.c_th_base = cdfParams.GetFloat("BaseThreshold", "Qualitative");
    params.c_th_min = cdfParams.GetFloat("MinThreshold", "Qualitative");
    params.c_th_fair = cdfParams.GetFloat("FairThreshold", "Qualitative");
    params.c_th_lost = cdfParams.GetFloat("LostThreshold", "Qualitative");
    
    params.c_mv_not = cdfParams.GetFloat("NoMovementThreshold", "Movement");
    params.c_mv_slow = cdfParams.GetFloat("SlowMovementThreshold", "Movement");
    
    return true;
}

void Tracker::setFrameTime(double dTime){
    for(unsigned i=0; i<m_modellist.size(); i++){
        m_modellist[i]->predictor->updateTime(dTime);
    }
}

bool Tracker::setCameraParameters(TomGine::tgCamera::Parameter cam_par){
    
    if(cam_par.zFar <= 0.0){
        ROS_DEBUG("[Tracker::setCameraParameters] Error 'Far Clipping Plane' not valid: %f", cam_par.zFar);
        return false;
    }
    if(cam_par.zNear < 0.0){
        ROS_DEBUG("[Tracker::setCameraParameters] Error 'Near Clipping Plane' not valid: %f", cam_par.zNear);
        return false;
    }
    
    if(cam_par.zNear >= cam_par.zFar){
        ROS_DEBUG("[Tracker::setCameraParameters] Error 'Clipping Panes' not valid: %f > %f", cam_par.zNear, cam_par.zFar);
        return false;
    }
    
    m_cam_perspective.Load(cam_par);
    return true;
}

bool Tracker::init(const char* trackINIFile, const char* camCalFile, const char* poseCalFile){
    
    if(m_tracker_initialized){
        ROS_DEBUG("[Tracker::init()] Warning: Tracker already initialized");
        return false;
    }
    
    // Load camera parameters
    if(!loadCamParsFromINI(camCalFile, poseCalFile))
        return false;
    
    // Load tracker parameter
    if(!loadTrackParsFromINI(trackINIFile))
        return false;
    
    // initialise tracker
    return init();
}

bool Tracker::init(const Parameter& trackParam){
    
    if(m_tracker_initialized){
        ROS_DEBUG("[Tracker::init()] Warning: Tracker already initialized");
        return false;
    }
    
    // load tracker parameter
    this->params = trackParam;
    // initialise tracker
    return init();
}

bool Tracker::init(){
    
    if(m_tracker_initialized){
        ROS_DEBUG("[Tracker::init()] Warning: Tracker already initialized");
        return false;
    }
    
    
    // OpenGL / devIL
    initGL();
    
    // Set pathes to file resources
    g_Resources->SetShaderPath(params.shaderPath.c_str());
    g_Resources->ShowLog(false);
    
    // Load camera parameter
    m_cam_perspective.Load(params.camPar);
    
    // Initialize Image Processor
    g_Resources->InitImageProcessor(params.camPar.width, params.camPar.height);
    m_ip = g_Resources->GetImageProcessor();
    
    // 	m_ip->avgInit(1024);
    
    // Textures
    m_tex_frame = new Texture();
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
    
    m_tex_frame_ip.push_back( new Texture() );
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
    for(int i=0; i<(int)params.num_spreadings-1; i++){
        m_tex_frame_ip.push_back( new Texture() );
        glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
    }
    
    if(params.num_spreadings == 0)
        params.m_spreadlvl = 0;
    else if(params.m_spreadlvl >= params.num_spreadings)
        params.m_spreadlvl = params.num_spreadings - 1;
    
    return (m_tracker_initialized = initInternal());
}

void Tracker::loadImage(unsigned char* image, GLenum format){
    m_tex_frame->load(image, params.camPar.width, params.camPar.height, format);
}

int Tracker::addModel(TomGine::tgModel& m, tgPose& p, std::string label, bool bfc){
    
    if(!m_tracker_initialized){
        char errmsg[128];
        sprintf(errmsg, "[Tracker::addModel()] Error tracker not initialised!");
        throw std::runtime_error(errmsg);
    }
    
    ModelEntry* modelEntry = new ModelEntry(m);
    modelEntry->label = label;
    modelEntry->model.setBFC(bfc);
    
    modelEntry->pose = p;
    modelEntry->initial_pose = p;
    
    // Calculate Zoom Direction and pass it to the particle filter
    TomGine::tgVector3 vCam = m_cam_perspective.GetPos();
    TomGine::tgVector3 vObj = TomGine::tgVector3(p.t.x, p.t.y, p.t.z);
    modelEntry->vCam2Model = vObj - vCam;
    modelEntry->predictor->setCamViewVector(modelEntry->vCam2Model);
    modelEntry->predictor->sample(modelEntry->distribution, params.num_particles, p, params.variation);
    modelEntry->predictor->setNoConvergence(params.pred_no_convergence);
    
    modelEntry->id = params.model_id_count++;
    modelEntry->num_particles = params.num_particles;
    modelEntry->num_recursions = params.num_recursions;
    m_modellist.push_back(modelEntry);

    return modelEntry->id;
}

int Tracker::addModelFromFile(const char* filename, tgPose& p, std::string label, bool bfc){
    if(!m_tracker_initialized){
        char errmsg[128];
        sprintf(errmsg, "[Tracker::addModelFromFile()] Error tracker not initialised!");
        throw std::runtime_error(errmsg);
    }

    ModelEntry* modelEntry = new ModelEntry();

    modelEntry->label = label;
    modelEntry->model.setBFC(bfc);
    
    ModelLoader modelloader;
    if(!modelloader.LoadPly(modelEntry->model, filename))
        return -1;
    modelEntry->pose = p;
    modelEntry->initial_pose = p;
    // Calculate Zoom Direction and pass it to the particle filter
    TomGine::tgVector3 vCam = m_cam_perspective.GetPos();
    TomGine::tgVector3 vObj = TomGine::tgVector3(p.t.x, p.t.y, p.t.z);
    modelEntry->vCam2Model = vObj - vCam;
    modelEntry->predictor->setCamViewVector(modelEntry->vCam2Model);
    modelEntry->predictor->sample(modelEntry->distribution, params.num_particles, p, params.variation);
    modelEntry->predictor->setNoConvergence(params.pred_no_convergence);
    
    modelEntry->id = params.model_id_count++;
    modelEntry->num_particles = params.num_particles;
    modelEntry->num_recursions = params.num_recursions;
    m_modellist.push_back(modelEntry);

    return modelEntry->id;
}

//BENCE: never called
//void Tracker::addPoseHypothesis(int id, tgPose &p, std::string label, bool bfc){
//	if(!m_tracker_initialized){
//		char errmsg[128];
//		sprintf(errmsg, "[Tracker::addPoseHypothesis()] Error tracker not initialised!");
//		throw std::runtime_error(errmsg);
//	}
//	ModelEntryList::iterator it = m_modellist.begin();
//	while(it != m_modellist.end()){
//		if(id==(*it)->id){
//			ModelEntry* modelEntry = new ModelEntry();
//			modelEntry->label = label;
//			modelEntry->model.setBFC(bfc);
//			modelEntry->model = m_modellist[id]->model;  // TODO maybe buggy
//			modelEntry->pose = p;
//			modelEntry->initial_pose = p;
//			modelEntry->predictor->sample(modelEntry->distribution, params.num_particles, p, params.variation);
//			modelEntry->predictor->setNoConvergence(params.pred_no_convergence);
//
//			modelEntry->id = params.hypotheses_id_count++;
//			modelEntry->num_particles = params.num_particles;
//			modelEntry->num_recursions = params.num_recursions;
//			modelEntry->hypothesis_id = id;
//			m_hypotheses.push_back(modelEntry);
//			return;
//		}
//		++it;
//	}
//	ROS_DEBUG("[Tracker::addHypothesis()] Warning model for hypothesis not found: %d - %d", id, (int)m_modellist.size());
//}

void Tracker::removeModel(int id){
    ModelEntryList::iterator it = m_modellist.begin();
    while(it != m_modellist.end()){
        if(id==(*it)->id){
            delete(*it);
            m_modellist.erase(it);
            return;
        }
        ++it;
    }
}

ModelEntry* Tracker::getModelEntry(int id){
    ModelEntryList::iterator it = m_modellist.begin();
    
    while(it != m_modellist.end()){
        if(id==(*it)->id){
            return (*it);
        }
        ++it;
    }
    return 0;
}

void Tracker::getModelPose(int id, tgPose& p){
    ModelEntryList::iterator it = m_modellist.begin();
    
    while(it != m_modellist.end()){
        if(id==(*it)->id){
            p = (*it)->pose;
            // 			p = (*it)->distribution.getParticle(0);
            return;
        }
        ++it;
    }
}

void Tracker::getModelPoseLPF(int id, tgPose& p){
    ModelEntryList::iterator it = m_modellist.begin();
    
    while(it != m_modellist.end()){
        if(id==(*it)->id){
            p = (*it)->pose;
            // 			p = (*it)->distribution.getParticle(0);
            return;
        }
        ++it;
    }
}

void Tracker::getModelPoseCamCoords(int id, tgPose& p){
    mat4 mv;
    mat3 R;
    vec3 t;
    
    mat3 gl2cv;
    gl2cv[0]=1.0; gl2cv[3]=0.0; 	gl2cv[6]=0.0;
    gl2cv[1]=0.0; gl2cv[4]=-1.0;	gl2cv[7]=0.0;
    gl2cv[2]=0.0; gl2cv[5]=0.0; 	gl2cv[8]=-1.0;
    
    
    ModelEntryList::iterator it = m_modellist.begin();
    
    while(it != m_modellist.end()){
        if(id==(*it)->id){
            m_cam_perspective.Activate();
            p = m_modellist[0]->pose;
            p.Activate();
            glGetFloatv(GL_MODELVIEW_MATRIX, mv);
            p.Deactivate();
            
            R = gl2cv * mat3(mv);
            t = gl2cv * vec3(mv[12], mv[13], mv[14]);
            
            // 			printf("mv: %f %f %f\n", mv[12], mv[13], mv[14]);
            // 			printf("t:  %f %f %f\n", t.x, t.y, t.z);
            
            p.SetPose(R,t);
            return;
        }
        ++it;
    }
}

void Tracker::getModelVelocity(int id, float &translational, float &angular){
    ModelEntryList::iterator it = m_modellist.begin();
    
    while(it != m_modellist.end()){
        if(id==(*it)->id){
            translational = (*it)->speed_translational;
            angular = (*it)->speed_angular;
            return;
        }
        ++it;
    }
}


void Tracker::getModelMovementState(int id, movement_state &m){
    ModelEntryList::iterator it = m_modellist.begin();
    
    while(it != m_modellist.end()){
        if(id==(*it)->id){
            m = (*it)->st_movement;
            return;
        }
        ++it;
    }
}

void Tracker::getModelQualityState(int id, quality_state &q){
    ModelEntryList::iterator it = m_modellist.begin();
    
    while(it != m_modellist.end())
    {
        if(id==(*it)->id)
        {
            q = (*it)->st_quality;
            return;
        }
        ++it;
    }
}

void Tracker::getModelConfidenceState(int id, confidence_state &q){
    ModelEntryList::iterator it = m_modellist.begin();
    
    while(it != m_modellist.end()){
        if(id==(*it)->id){
            q = (*it)->st_confidence;
            return;
        }
        ++it;
    }
}

void Tracker::drawTest(){
    tgPose p;
    mat4 mv;
    mat3 R, Rinv;
    vec3 t;
    
    std::vector<vec3> v;
    
    m_cam_perspective.Activate();
    
    if(!m_modellist.empty()){
        
        TomGine::tgModel model = m_modellist[0]->model;
        for(unsigned i=0; i<model.getVertexSize(); i++){
            v.push_back(model.getVertex(i).pos);
        }		
        
        p = m_modellist[0]->pose;
        
        p.Activate();
        glGetFloatv(GL_MODELVIEW_MATRIX, mv);
        p.Deactivate();
        
        R = mat3(mv);
        t = vec3(mv[12], mv[13], mv[14]);
        p.SetPose(R,t);
        
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        glDisable(GL_DEPTH_TEST);
        
        glPointSize(5.0);
        glBegin(GL_POINTS);
        glColor3f(1.0f, 0.0f, 0.0f);
        glVertex3f(t.x, t.y, t.z);			
        glEnd();
        
        glPointSize(2.0);
        
        glBegin(GL_POINTS);
        glColor3f(1.0f, 0.0f, 0.0f);
        
        for(unsigned int i=0; i<v.size(); i++){
            v[i] = (R * v[i]) + t;
            
            glVertex3f(v[i].x, v[i].y, v[i].z);			
        }
        glEnd();
        
    }
}

void Tracker::getModelInitialPose(int id, tgPose& p){
    ModelEntryList::iterator it = m_modellist.begin();
    
    while(it != m_modellist.end()){
        if(id==(*it)->id){
            p = (*it)->initial_pose;
            return;
        }
        ++it;
    }
}

void Tracker::getModelConfidence(int id, float& c){
    ModelEntryList::iterator it = m_modellist.begin();
    
    while(it != m_modellist.end()){
        if(id==(*it)->id){
            c = (*it)->pose.c;
            return;
        }
        ++it;
    }
}

void Tracker::getModelConfidenceEdge(int id, float& c){
    ModelEntryList::iterator it = m_modellist.begin();
    
    while(it != m_modellist.end()){
        if(id==(*it)->id){
            c = (*it)->confidence_edge;
            return;
        }
        ++it;
    }
}

void Tracker::getModelConfidenceColor(int id, float& c){
    ModelEntryList::iterator it = m_modellist.begin();
    
    while(it != m_modellist.end()){
        if(id==(*it)->id){
            c = (*it)->confidence_color;
            return;
        }
        ++it;
    }
}

bool Tracker::getModelPoint3D(int id, int x_win, int y_win, double& x3, double& y3, double& z3){
    ModelEntryList::iterator it;
    
    for(it = m_modellist.begin(); it < m_modellist.end(); ++it){
        if(id==(*it)->id){
            // Activate Camera
            TomGine::tgCamera cam = m_cam_perspective;
            cam.SetZRange(0.0, 1.0);
            cam.Activate();
            
            // Clear Depth Buffer
            glClear(GL_DEPTH_BUFFER_BIT);
            glEnable(GL_DEPTH_TEST);
            
            // Apply pose
            (*it)->pose.Activate();
            
            // Draw Model Faces
            m_lighting.Activate();
            (*it)->model.drawFaces();
            m_lighting.Deactivate();
            
            // ************************
            int viewport[4];
            double modelview[16];
            double projection[16];
            double result[3];
            
            glGetDoublev(GL_MODELVIEW_MATRIX, &modelview[0] );
            glGetDoublev(GL_PROJECTION_MATRIX, &projection[0] );
            glGetIntegerv(GL_VIEWPORT, &viewport[0] );
            y_win = viewport[3] - y_win;	// flip y for OpenGL
            
            // Read value of depth buffer at position (x_win, y_win)
            glReadPixels(x_win, y_win, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &z3 );
            
            if(z3 > 0.99)
                return false;
            
            // calculate intersection of camera viewing vector and model surface
            gluUnProject((double)x_win, (double)y_win, (double)z3, modelview, projection, viewport, &result[0], &result[1], &result[2]); 
            
            x3 = result[0];
            y3 = result[1];
            z3 = result[2];
            
            (*it)->pose.Deactivate();
            
            return true;			
        }
    }
    return false;
}

void Tracker::getModelMask(int id, Texture &mask){
    ModelEntryList::iterator it;
    
    for(it = m_modellist.begin(); it < m_modellist.end(); ++it){
        if(id==(*it)->id){
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            m_cam_perspective.Activate();
            m_lighting.Deactivate();
            glColor3f(1,1,1);
            glEnable(GL_DEPTH_TEST); glDepthMask(1);
            (*it)->pose.Activate();
            (*it)->model.DrawFaces();
            (*it)->pose.Deactivate();
            glDisable(GL_DEPTH_TEST); glDepthMask(0);
            
            mask.copyTexImage2D(params.camPar.width, params.camPar.height);
        }
    }
}

void Tracker::getModelEdgeMask(int id, Texture &mask){
    ModelEntryList::iterator it;
    
    for(it = m_modellist.begin(); it < m_modellist.end(); ++it){
        if(id==(*it)->id){
            computeModelEdgeMask((*it), mask);
        }
    }
}

bool  Tracker::getModelMaskOwnEdges(int id){
    ModelEntryList::iterator it;
    
    for(it = m_modellist.begin(); it < m_modellist.end(); ++it){
        if(id==(*it)->id){
            return (*it)->mask_geometry_edges;
        }
    }
    return 0;
}

void Tracker::setModelInitialPose(int id, tgPose& p){
    ModelEntryList::iterator it = m_modellist.begin();
    
    while(it != m_modellist.end()){
        if(id==(*it)->id){
            (*it)->setInitialPose(p, 0.2f, 0.2f);
            return;
        }
        ++it;
    }
}

void Tracker::setModelPredictor(int id, Predictor* predictor){
    ModelEntryList::iterator it = m_modellist.begin();
    while(it != m_modellist.end()){
        if(id==(*it)->id){
            (*it)->predictor = predictor;			
            return;
        }
        ++it;
    }
}

void Tracker::setModelPredictorNoConvergence(int id, float no_conv){
    ModelEntryList::iterator it = m_modellist.begin();
    while(it != m_modellist.end()){
        if(id==(*it)->id){
            (*it)->predictor->setNoConvergence(no_conv);			
            return;
        }
        ++it;
    }
}

void Tracker::setModelMask(int id, Texture *mask){
    
}

void Tracker::setModelMaskOwnEdges(int id, bool masked){
    ModelEntryList::iterator it = m_modellist.begin();
    while(it != m_modellist.end()){
        if(id==(*it)->id){
            (*it)->mask_geometry_edges = masked;			
            return;
        }
        ++it;
    }
}

void Tracker::setModelLock(int id, bool lock){
    ModelEntryList::iterator it = m_modellist.begin();
    while(it != m_modellist.end()){
        if(id==(*it)->id){
            (*it)->lock = lock;
            return;
        }
        ++it;
    }
}

void  Tracker::setModelRecursionsParticle(int id, int num_recursions, int num_particles){
    ModelEntryList::iterator it = m_modellist.begin();
    while(it != m_modellist.end()){
        if(id==(*it)->id){
            (*it)->num_recursions = num_recursions;
            (*it)->num_particles = num_particles;
            return;
        }
        ++it;
    }
}

void Tracker::saveModel(int id, const char* pathname){
    ModelLoader modelloader;
    ModelEntryList::iterator it = m_modellist.begin();
    string name;
    while(it != m_modellist.end()){
        if(id==(*it)->id){
            name = string(pathname);
            name.append((*it)->label);
            modelloader.SavePly((*it)->model, name.c_str());
            return;
        }
        ++it;
    }
}

void Tracker::saveModels(const char* pathname){
    ModelLoader modelloader;
    ModelEntryList::iterator it = m_modellist.begin();
    string name;
    while(it != m_modellist.end()){
        name = string(pathname);
        name.append((*it)->label);
        (*it)->model.unwarpTexturesBox_hacky(name.c_str());
        modelloader.SavePly((*it)->model, name.c_str());
        ++it;
    }
}

void Tracker::saveScreenshot(const char* filename){
    IplImage* img = cvCreateImage ( cvSize ( params.camPar.width, params.camPar.height ), IPL_DEPTH_8U, 3 );
    glReadPixels(0,0,params.camPar.width,params.camPar.height,GL_RGB,GL_UNSIGNED_BYTE, img->imageData);
    cvConvertImage(img, img, CV_CVTIMG_FLIP | CV_CVTIMG_SWAP_RB);
    cvSaveImage(filename, img);
    cvReleaseImage(&img);
}

cv::Mat Tracker::getImage()
{
    IplImage* img = cvCreateImage ( cvSize ( params.camPar.width, params.camPar.height ), IPL_DEPTH_8U, 3 );
    glReadPixels(0,0,params.camPar.width,params.camPar.height,GL_RGB,GL_UNSIGNED_BYTE, img->imageData);
    cvConvertImage(img, img, CV_CVTIMG_FLIP | CV_CVTIMG_SWAP_RB);
    //FIXME: result(img) does not take ownership of the memory
    cv::Mat result(img, true);
    cvReleaseImage(&img);
    return result;
}

void Tracker::setLockFlag(bool val){
    ModelEntryList::iterator it = m_modellist.begin();
    while(it != m_modellist.end()){
        (*it)->lock = val;
        ++it;
    }
    m_lock = val;
}

void Tracker::drawModel(tgPose p){
    m_cam_perspective.Activate();
    
    glEnable(GL_DEPTH_TEST);
    glClear(GL_DEPTH_BUFFER_BIT);
    
    p.Activate();
    
    for(unsigned i=0; i<m_modellist.size(); i++){
        
        glColorMask(0,0,0,0);
        m_modellist[i]->model.drawFaces();
        glColorMask(1,1,1,1);
        m_modellist[i]->model.drawEdges();
    }
    
    p.Deactivate();
}

void Tracker::drawModel(const TomGine::tgModel &m, const tgPose &p){
    m_cam_perspective.Activate();
    
    m_lighting.Activate();
    p.Activate();
    
    m.DrawFaces();
    
    p.Deactivate();
    m_lighting.Deactivate();
}

void Tracker::drawModelWireframe(const TomGine::tgModel &m, const tgPose &p, float linewidth){
    
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glLineWidth(linewidth);
    
    m_cam_perspective.Activate();
    
    m_lighting.Deactivate();
    
    p.Activate();
    
    m.DrawFaces();
    
    p.Deactivate();
    
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
}

void Tracker::drawCoordinateSystem(float linelength, float linewidth, TomGine::tgPose pose)
{
  //2012-11-27: Added by jordi to prevent painting the coordinate system.
  //If the function is simply not called then the wired view of the tracked object does
  //not appears properly.
  bool ENABLE_LINES = false;

    glDisable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
    m_cam_perspective.Activate();
    
    float l = linelength;
    glLineWidth(linewidth);
    
    vec3 o = pose.t;
    
    vec3 x = pose * vec3(l,0,0);
    vec3 y = pose * vec3(0,l,0);
    vec3 z = pose * vec3(0,0,l);
    
    glPushMatrix();
    glColor3f(1,0,0);
    if ( ENABLE_LINES )
    {
      glBegin(GL_LINES);
      glVertex3f(o.x, o.y, o.z);
      glVertex3f(x.x, x.y, x.z);
      glEnd();
    }
    
    glColor3f(0,1,0);
    if ( ENABLE_LINES )
    {
      glBegin(GL_LINES);
      glVertex3f(o.x, o.y, o.z);
      glVertex3f(y.x, y.y, y.z);
      glEnd();
    }
    
    glColor3f(0,0,1);
    if ( ENABLE_LINES )
    {
      glBegin(GL_LINES);
      glVertex3f(o.x, o.y, o.z);
      glVertex3f(z.x, z.y, z.z);
      glEnd();
    }

    glPopMatrix();
}


// render coordinate frame
void Tracker::drawCoordinates(float linelength){
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
    m_cam_perspective.Activate();
    
    //	float l1 = 0.06f;
    //	float l2 = 0.02f;
    //	float b1 = 0.001f;
    //	float b2 = 0.003f;
    
    drawCoordinateSystem(linelength, 2.0f);
    
    // 	// X - Axis
    // 	glPushMatrix();
    // 		glColor3f(1.0,0.0,0.0);
    // 		glBegin(GL_TRIANGLE_FAN);
    // 			glVertex3f(l1,		0.0,	 b1);
    // 			glVertex3f(0.0,		0.0,	 b1);
    // 			glVertex3f(0.0,		0.0,	-b1);
    // 			glVertex3f(l1,		0.0,	-b1);
    // 			glVertex3f(l1,		0.0,	-b1-b2);
    // 			glVertex3f(l1+l2,	0.0,	0.0);
    // 			glVertex3f(l1,		0.0,	 b1+b2);
    // 		glEnd();
    // 	glPopMatrix();
    // 		
    // 	// Y - Axis
    // 	glPushMatrix();
    // 		glColor3f(0.0,1.0,0.0);
    // 		glRotatef(90, 0.0, 0.0, 1.0);
    // 		glBegin(GL_TRIANGLE_FAN);
    // 			glVertex3f(l1,		0.0,	 b1);
    // 			glVertex3f(0.0,		0.0,	 b1);
    // 			glVertex3f(0.0,		0.0,	-b1);
    // 			glVertex3f(l1,		0.0,	-b1);
    // 			glVertex3f(l1,		0.0,	-b1-b2);
    // 			glVertex3f(l1+l2,	0.0,	0.0);
    // 			glVertex3f(l1,		0.0,	 b1+b2);
    // 		glEnd();
    // 	glPopMatrix();
    // 	
    // 	// Z - Axis
    // 	glPushMatrix();
    // 		glColor3f(0.0,0.0,1.0);
    // 		glRotatef(-90, 0.0, 1.0, 0.0);
    // 		glBegin(GL_TRIANGLE_FAN);
    // 			glVertex3f(l1,		0.0,	 b1);
    // 			glVertex3f(0.0,		0.0,	 b1);
    // 			glVertex3f(0.0,		0.0,	-b1);
    // 			glVertex3f(l1,		0.0,	-b1);
    // 			glVertex3f(l1,		0.0,	-b1-b2);
    // 			glVertex3f(l1+l2,	0.0,	0.0);
    // 			glVertex3f(l1,		0.0,	 b1+b2);
    // 		glEnd();
    // 	glPopMatrix();
    
}

void Tracker::drawImage(unsigned char* image){
    glDepthMask(0);
    glColor3f(1.0,1.0,1.0);
    
    if(image == NULL){
        if(m_draw_edges)
            m_ip->render(m_tex_frame_ip[params.m_spreadlvl]);
        else
            m_ip->render(m_tex_frame);
    }else{
        m_tex_frame->load(image, params.camPar.width, params.camPar.height);
        m_ip->flipUpsideDown(m_tex_frame, m_tex_frame);
        m_ip->render(m_tex_frame);
    }
    
    glDepthMask(1);
}

void Tracker::drawCalibrationPattern(float point_size){
    
    if(m_calib_points.empty())
        return;
    
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
    
    m_cam_perspective.Activate();
    
    glPointSize(point_size);
    
    glBegin(GL_POINTS);
    
    for( unsigned i=0; i<m_calib_points.size(); i++ )
        glVertex3f(m_calib_points[i].x, m_calib_points[i].y, m_calib_points[i].z);
    
    glEnd();
}

void Tracker::loadCalibrationPattern(const char* mdl_file){
    FILE* pfile = 0;
    
    pfile = fopen(mdl_file, "r");
    
    if(pfile!=NULL)	{
        unsigned n=0;
        vec3 p;
        rewind(pfile);
        int size = fscanf(pfile, "%u", &n);
        m_calib_points.clear();
        for(unsigned i=0; i<n; i++){
            size = fscanf(pfile, "%f %f %f", &p.x, &p.y, &p.z);
            m_calib_points.push_back(p);

        }
        printf("size: %d", size);
    }else{
        ROS_DEBUG("[Tracker::loadCalibrationPattern] Warning couldn't load model file '%s'\n", mdl_file);
    }
    
    fclose(pfile);
}

void Tracker::drawPixel(float u, float v, vec3 color, float size){
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);
    m_ip->setCamOrtho();
    
    glPointSize(size);
    glColor3f(color.x, color.y, color.z);
    
    glBegin(GL_POINTS);
    glVertex3f(u,v,0.0);
    glEnd();
}

void Tracker::drawPoint(float x, float y, float z, float size){
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);
    m_cam_perspective.Activate();
    
    glPointSize(size);
    // 	glColor3f(1.0,0.0,0.0);
    
    glBegin(GL_POINTS);
    glVertex3f(x,y,z);
    glEnd();
}

void Tracker::getGlError(){
    int err = glGetError();
    switch(err){
    case GL_NO_ERROR:
        break;
    case GL_INVALID_ENUM:
        ROS_DEBUG("glGetError: GL_INVALID_ENUM\n");
        break;
    case GL_INVALID_VALUE:
        ROS_DEBUG("glGetError: GL_INVALID_VALUE\n");
        break;
    case GL_INVALID_OPERATION:
        ROS_DEBUG("glGetError: GL_INVALID_OPERATION\n");
        break;
    case GL_STACK_OVERFLOW:
        ROS_DEBUG("glGetError: GL_STACK_OVERFLOW\n");
        break;
    case GL_STACK_UNDERFLOW:
        ROS_DEBUG("glGetError: GL_STACK_UNDERFLOW\n");
        break;
    case GL_OUT_OF_MEMORY:
        ROS_DEBUG("glGetError: GL_OUT_OF_MEMORY\n");
        break;
    default:
        ROS_DEBUG("glGetError: no known error\n");
        break;
    }
}

void Tracker::computeModelEdgeMask(ModelEntry* modelEntry, Texture &mask)
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    m_cam_perspective.Activate();
    m_lighting.Activate();
    glEnable(GL_DEPTH_TEST); glDepthMask(1);
    modelEntry->pose.Activate();
    modelEntry->model.DrawFaces();
    modelEntry->pose.Deactivate();
    glDisable(GL_DEPTH_TEST); glDepthMask(0);
    m_lighting.Deactivate();
    
    mask.copyTexImage2D(params.camPar.width, params.camPar.height);
    m_ip->sobel(&mask, &mask, 0.01, true, true);
}


// Show performance and likelihood
void Tracker::printStatistics(){
    printf("\n\nStatistics: \n");
    
    for(unsigned i=0; i<m_modellist.size(); i++){
        //tgPose pMean =  m_modellist[i]->distribution.getMean();
        printf("	Object %u '%s'\n", i, m_modellist[i]->label.c_str());
        printf("		FPS: %.1f\n", 1.0f/m_ftime);
        printf("		Textured: %d\n", m_modellist[i]->model.m_textured);
        printf("		Recursions: %i\n", m_modellist[i]->num_recursions );
        printf("		Particles: %i\n", m_modellist[i]->distribution.size() );
        printf("		Pose: %f %f %f\n", m_modellist[i]->pose.t.x, m_modellist[i]->pose.t.y, m_modellist[i]->pose.t.z);
        printf("		Variance: %f \n", m_modellist[i]->distribution.getVariancePt() );
        printf("		Confidence: %f \n", m_modellist[i]->pose.c);
        printf("		Weight: %f \n", m_modellist[i]->distribution.getMaxW());
        printf("		Spread: %u / %u\n", params.m_spreadlvl, params.num_spreadings);
        printf("		Kernel: %d \n", params.kernel_size);
        printf("		ImageSobelTh: %f \n", params.image_sobel_th);
        printf("		ModelSobelTh: %f \n", params.model_sobel_th);
    }
}

void Tracker::reset()
{
    for(unsigned i=0; i<m_modellist.size(); i++)
    {
        m_modellist[i]->predictor->sample(
                m_modellist[i]->distribution, params.num_particles,
                m_modellist[i]->initial_pose, params.variation);
        m_modellist[i]->pose = m_modellist[i]->initial_pose;
        // TODO evaluate Distribution::v_max (for the right confidence value when the model is locked)
        //m_modellist[i]->m_lpf_cl.Set(0.0f);        
    }
}

void Tracker::reset(int id)
{
    if(id >= 0 && id<static_cast<int>(m_modellist.size()))
    {
        m_modellist[id]->predictor->sample(
                m_modellist[id]->distribution, params.num_particles,
                m_modellist[id]->initial_pose, params.variation);
        m_modellist[id]->pose = m_modellist[id]->initial_pose;
        //m_modellist[id]->m_lpf_cl.Set(0.0f);
    }
}

void Tracker::resetUnlockLock()
{
    this->reset();
    this->setLockFlag(false);
    const unsigned int num_frames(30);
    for(unsigned i=0; i<num_frames; i++)
        this->track();
    this->setLockFlag(true);
}



