
#include <blort/ThreadObject/TomGineThread.h>
#include <blort/TomGine/tgEngine.h>
#include <blort/TomGine/tgShapeCreator.h>
#include <stdexcept>

using namespace TomGine;
using namespace std;

CTomGineThread::CTomGineThread(int width, int height)
{
	m_mutex.Lock();
		m_quit = false;
		m_use_campars = false;
		m_width = width;
		m_height = height;
	m_mutex.Unlock();
}

CTomGineThread::CTomGineThread(int width, int height, const TomGine::tgCamera::Parameter& tgCamParams)
{
	m_mutex.Lock();
		m_quit = false;
		m_use_campars = true;
		m_width = width;
		m_height = height;
		m_camPars = tgCamParams;
	m_mutex.Unlock();
}

CTomGineThread::~CTomGineThread()
{
	m_mutex.Lock();
		m_quit = true;
	m_mutex.Unlock();
	
	m_running.Wait();
}

void CTomGineThread::SetModel(const TomGine::tgModel& model)
{
	m_mutex.Lock();
	
		m_max_vertex_length = 0.0f;
		float l;
		for(unsigned int i=0; i<model.m_vertices.size(); i++){
			l = model.m_vertices[i].pos.length();
			
			if( l > m_max_vertex_length )
				m_max_vertex_length = l;
		}
		
		m_model = model;
	m_mutex.Unlock();
}

void CTomGineThread::SetPose(const TomGine::tgPose& pose)
{
	m_mutex.Lock();
		m_pose = pose;
	m_mutex.Unlock();
}

void CTomGineThread::AddSifts(const std::vector<blortRecognizer::Siftex>& sl)
{
	m_mutex.Lock();
		m_lastsiftexlist = sl;
		for(unsigned i=0; i<sl.size(); i++)
		  m_siftexlist.push_back(sl[i]);
	m_mutex.Unlock();
	
	double pos[3] = {0,0,0};
	double normal[3] = {0,0,0};
	double view[3] = {0,0,0};
	unsigned sc = sl.size();
	
	for(unsigned i=0; i<sc; i++){
	  pos[0] += sl[i].pos.x;
	  pos[1] += sl[i].pos.y;
	  pos[2] += sl[i].pos.z;
	  normal[0] += sl[i].normal.x;
	  normal[1] += sl[i].normal.y;
	  normal[2] += sl[i].normal.z;
	  view[0] += sl[i].viewray.x;
	  view[1] += sl[i].viewray.y;
	  view[2] += sl[i].viewray.z;
	}
	
	vec3 vPos = vec3(float(pos[0]/sc), float(pos[1]/sc), float(pos[2]/sc)); 
	//vec3 vNormal = vec3(float(normal[0]/sc), float(normal[1]/sc), float(normal[2]/sc)); 
	vec3 vView = vec3(float(view[0]/sc), float(view[1]/sc), float(view[2]/sc));
	
	vec3 p = vPos - vView;
    //vec3 p = sl[0].pos - sl[0].viewray;
	
	vec3 f = vView;	f.normalize();
	vec3 u = vec3(0,0,1);
	vec3 s;
	s.cross(u,f); s.normalize();
	u.cross(f,s); u.normalize();

	float rad = 0.0f;
	for(unsigned i=0; i<sc; i++){
		vec3 d = vPos - sl[i].pos;
		float l = d.length();
		if(rad < l)
		  rad=l;
	}
	m_viewscale.push_back(vec3(rad, rad, vView.length()));

	float fR[9] = { s.x, s.y, s.z,
					u.x, u.y, u.z,
					f.x, f.y, f.z};
	mat3 R = mat3(fR);
	TomGine::tgPose pose;
	pose.SetPose(R, p);
	m_viewlist.push_back(pose);
}

void CTomGineThread::AddView(const TomGine::tgPose& view)
{
// 	m_mutex.Lock();
// 		m_viewlist.push_back(view);
// 	m_mutex.Unlock();
}

BOOL CTomGineThread::OnTask()
{
	m_running.Lock();
	
	m_mutex.Lock();
		tgEngine m_engine(m_width, m_height, 5.0f, 0.01f, "TomGine", true);
		
		if(m_use_campars)
		{
			TomGine::tgCamera cam;
			cam.Load(m_camPars);
			m_engine.SetCamera(cam);
			m_engine.UpdateCameraViews(cam);
		}
	m_mutex.Unlock();
	
	tgMaterial matBlueBlend;
	matBlueBlend.Color(0.0f, 0.0f, 1.0f, 0.5f);
	m_cone.m_material = matBlueBlend;
	
	tgShapeCreator m_shape_creator;
	m_shape_creator.CreateSphere(m_sphere, 0.1, 5, ICOSAHEDRON);
	m_shape_creator.CreateCone(m_cone, 1.0, 1.0, 16, 1, false);
// 	tgModel &model, float radius, float height, int slices, int stacks, bool closed
	
	glClearColor(1,1,1,0);
	
	while(!m_quit)
	{
		m_mutex.Lock();
			
			// Draw model
			m_pose.Activate();
				m_model.DrawFaces();
				
// 				for(unsigned i=0; i<m_viewlist.size(); i++)
// 				{
// 					m_viewlist[i].Activate();
// 					m_cone.DrawFaces();
// 				}
			
			// TODO test if Siftex is visible
			
			
			// Draw points
			glColor3f(0.0f,1.0f,0.0f);
			glDisable(GL_LIGHTING);
			glPointSize(2);
			glBegin(GL_POINTS);
			for(unsigned int i=0; i<m_siftexlist.size(); i++)
			{
				glVertex3f(m_siftexlist[i].pos.x, m_siftexlist[i].pos.y, m_siftexlist[i].pos.z);
			}
			glEnd();
			glEnable(GL_LIGHTING);
			
			// Draw normals
			glColor3f(0.0f, 0.0f, 1.0f);
			glDisable(GL_LIGHTING);
			glBegin(GL_LINES);
			for(unsigned int i=0; i<m_lastsiftexlist.size(); i++)
			{
				glVertex3f(m_lastsiftexlist[i].pos.x, m_lastsiftexlist[i].pos.y, m_lastsiftexlist[i].pos.z);
				glVertex3f(	m_lastsiftexlist[i].pos.x - m_lastsiftexlist[i].viewray.x,
										m_lastsiftexlist[i].pos.y - m_lastsiftexlist[i].viewray.y,
										m_lastsiftexlist[i].pos.z - m_lastsiftexlist[i].viewray.z);
			}
			glEnd();
			glEnable(GL_LIGHTING);
			
			for(unsigned i=0; i<m_viewlist.size(); i++)
			{
			  m_viewlist[i].Activate();
			  
			  glPushMatrix();
			  glScalef(m_viewscale[i].x,m_viewscale[i].y,m_viewscale[i].z);
			  glEnable(GL_BLEND);
			  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
			  m_cone.DrawFaces();
			  glDisable(GL_BLEND);
			  glPopMatrix();
			  
			  m_viewlist[i].Deactivate();
			}
			
			m_pose.Deactivate();
		m_mutex.Unlock();
		
		Sleep(10);
		
		m_engine.Update();
	}

	m_running.Unlock();
	return TRUE;
}




