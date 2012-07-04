//SOURCE:
//			Title:			class tgEngine
//			File:				tgEngine.cpp
//
//			Function:		Main file of Engine providing interfaces
//
//			Author:			Thomas Mörwald
//			Date:				20.11.2009
// ----------------------------------------------------------------------------

#include <blort/TomGine/tgEngine.h>

using namespace TomGine;
using namespace blortGLWindow;

tgEngine::tgEngine(	unsigned width,
					unsigned height,
					float far,
					float near,
					const char* name,
					bool bfc)
{
	m_window = new blortGLWindow::GLWindow(width, height, name);
	
	m_width = width;
	m_height = height;
	m_far = far;
	m_near = near;
	m_bfc = bfc;
	
	m_frametime = 0.0;
	
	m_input_rotation_speed		= 1.0f;
	m_input_translation_speed	= 1.0f;
	m_input_zoom_speed			= 1.0f;
	
	m_cor = tgVector3(0.0,0.0,0.0);
	
	m_button_left = false;
	m_button_middle = false;
	m_button_right = false;
	
	m_wireframe = false;
	m_smoothshading = false;
	m_show_background_image = false;
	
	m_mouse_pos[0] = 0;
	m_mouse_pos[1] = 0;
	
	float da = 0.25f*(m_far-m_near);
	// Setup 3D camera
	m_camera.Set(	da, da, da,							// Position of camera
					0.0f, 0.0f, 0.0f,					// Point where camera looks at
					0.0f, 1.0f, 0.0f,					// UP-Vector of Camera
					45, width, height,					// field of view in degree in y, image width, image height
					near, far,							// near clipping plane, far clipping plane
					GL_PERSPECTIVE);					// Perspective camera
	UpdateCameraViews(m_camera);
	
	// Setup 2D camera
	m_cam_ortho.Set(	0.0f, 0.0f, 1.0f,
						0.0f, 0.0f, 0.0f,
						0.0f, 1.0f, 0.0f,
						45, width, height,
						0.1f, 2.0f,
						GL_ORTHO);
	
	tgVector3 cam_f = m_camera.GetF();
	m_light0.ambient = vec4(0.4f,0.4f,0.4f,1.0f);
	m_light0.diffuse = vec4(1.0f,1.0f,1.0f,1.0f);
	m_light0.specular = vec4(1.0f,1.0f,1.0f,1.0f);
	m_light0.position = vec4(-cam_f.x, -cam_f.y, -cam_f.z, 1.0f);
	m_lighting.ApplyLight(m_light0,0);
	
#ifdef USE_FTGL_FONT
//  	m_font_tf = new FTTextureFont(TTF_FONT);
// 	if(m_font_tf->Error()){
// 		printf("[tgEngine::tgEngine] Warning Font not correct '%s'\n", TTF_FONT);
// 		m_font_tf = 0;
// 	}else{
// 		m_font_tf->FaceSize(18);
// 		printf("[tgEngine::tgEngine] Font used: '%s'\n", TTF_FONT);
// 	}
#endif
	
	m_background_image = 0;
// 	Welcome();
}

tgEngine::~tgEngine(){
	if(m_window) delete(m_window);
	if(m_background_image) delete(m_background_image);
}

void tgEngine::Welcome(){
	printf("\n   ***   TomGine Render Engine   ***\n\n");
	printf("rotate: left mouse button\n");
	printf("slide:  right mouse button\n");
	printf("zoom:   mouse wheel\n");
	printf("[f]:    toggle shading mode\n");
	printf("[w]:    draw wireframed\n");
	printf("[esc]:	quit\n");
	printf("\n");
}

bool tgEngine::Update(){
	float fTime;
	std::vector<Event> eventlist;
	return Update(fTime, eventlist);
}

bool tgEngine::Update(float &fTime){
	std::vector<Event> eventlist;
	return Update(fTime, eventlist);
}

bool tgEngine::Update(std::vector<Event> &eventlist){
	float fTime;
	return Update(fTime, eventlist);
}
	
bool tgEngine::Update(float &fTime, std::vector<Event> &eventlist){
	
	// User input handling (keyboard, mouse)
	bool quit = true;
	Event event;
	while(m_window->GetEvent(event)){
		quit = InputControl(event);
		eventlist.push_back(event);
// 		if(quit==false)
// 			return false;
	}
	
	Activate3D();
// 	DrawCoordinates();
	
	Swap();
	
	// update frametime
	fTime = m_frametime = (float)m_timer.Update();

	// clear framebuffer and depth buffer
	glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
	
	DrawBackgroundImage();
	
	Activate3D();
	
	// Light pointing along camera viewing axis
	tgLight light;
	tgVector3 cam_f = m_camera.GetF();
	light.ambient = vec4(0.4f,0.4f,0.4f,1.0f);
	light.diffuse = vec4(1.0f,1.0f,1.0f,1.0f);
	light.specular = vec4(1.0f,1.0f,1.0f,1.0f);
	light.position = vec4(-cam_f.x, -cam_f.y, -cam_f.z, 1.0f);
	m_lighting.ApplyLight(light,0);

	return quit;
}

bool tgEngine::InputControl(Event &event){
	int x_rel = 0;
	int y_rel = 0;
	
	switch(event.type){
	
		// *********************************************************
		//			KeyCode:		/usr/include/X11/keysymdef.h
		case TMGL_Press:
// 			printf("event.key.keysym: %x\n", event.key.keysym);
			switch(event.input){
				case TMGL_Escape:
					return false;
					break;
				case TMGL_f:
					if(m_smoothshading)
						glShadeModel(GL_SMOOTH);
					else
						glShadeModel(GL_FLAT);
					m_smoothshading = !m_smoothshading;
					break;
				case TMGL_p:
					p_pressed = true;
					break;
				case TMGL_w:
					if(m_wireframe)
						glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
					else
						glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
					m_wireframe = !m_wireframe;
					break;
				case TMGL_KP_0:
				case TMGL_z:
					m_camera = m_cam[0];
					break;
				case TMGL_KP_7:
					m_camera = m_cam[1];
					break;
				case TMGL_KP_6:
					m_camera = m_cam[2];
					break;
				case TMGL_KP_4:
					m_camera = m_cam[3];
					break;
				case TMGL_KP_2:
					m_camera = m_cam[4];
					break;
				case TMGL_KP_8:
					m_camera = m_cam[5];
					break;
				default:
					break;
					
				case TMGL_Button1:
					m_button_left = true;
					break;
				case TMGL_Button2:
					m_button_middle = true;
					break;
				case TMGL_Button3:
					m_button_right = true;
					break;
				case TMGL_Button4:
					m_camera.TranslateF(0.01f*(m_far-m_near)*m_input_translation_speed);
					break;
				case TMGL_Button5:
					m_camera.TranslateF(-0.01f*(m_far-m_near)*m_input_translation_speed);
					break;
			}
			break;
			
		// *********************************************************
		case TMGL_Release:
			switch(event.input){
				case TMGL_Button1:
					m_button_left = false;
					break;
				case TMGL_Button2:
					m_button_middle = false;
					break;
				case TMGL_Button3:
					m_button_right = false;
					break;
                        default:
                                        break;
			}
			break;
			
		// *********************************************************
		case TMGL_Motion:
			x_rel = event.motion.x - m_mouse_pos[0];
			y_rel = event.motion.y - m_mouse_pos[1];
			
			if(m_button_left){
				m_camera.Orbit(m_cor, m_camera.GetU(), -0.05f * x_rel * m_input_rotation_speed);
				m_camera.Orbit(m_cor, m_camera.GetS(), -0.05f * y_rel * m_input_rotation_speed);					
			}else if(m_button_right){
				m_camera.TranslateS(-0.0005f*(m_far-m_near)*x_rel * m_input_zoom_speed);
				m_camera.TranslateU(0.0005f*(m_far-m_near)*y_rel * m_input_zoom_speed);
			}
			
			m_mouse_pos[0] = event.motion.x;
			m_mouse_pos[1] = event.motion.y;
			
			break;
			
		// *********************************************************
		case TMGL_Expose:
			m_width = event.expose.width; m_height = event.expose.height;
			m_camera.SetViewport(m_width, m_height);
			m_cam[0].SetViewport(m_width, m_height);
			m_cam[1].SetViewport(m_width, m_height);
			m_cam[2].SetViewport(m_width, m_height);
			m_cam[3].SetViewport(m_width, m_height);
			m_cam[4].SetViewport(m_width, m_height);
			m_cam[5].SetViewport(m_width, m_height);
			m_cam_ortho.Set(0.0f, 0.0f, 1.0f,
							0.0f, 0.0f, 0.0f,
							0.0f, 1.0f, 0.0f,
							45, m_width, m_height,
							0.1f, 2.0f,
							GL_ORTHO);
			break;
			
		// *********************************************************
// 		case ClientMessage:
// 			if(event.clientmessage.stop)
// 				return false;
// 			break;
                default:
                        break;
			
	} // switch(event.type)
return true;
}

void tgEngine::DrawCoordinates(){
	float l1 = 0.01f*(m_far-m_near);
	m_lighting.Deactivate();
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_TEXTURE_2D);
		
	glBegin(GL_LINES);
		glColor3f(1.0f,0.0f,0.0f);
		glVertex3f(0.0f,0.0f,0.0f); glVertex3f(l1, 0.0f, 0.0f);
		glColor3f(0.0f,1.0f,0.0f);
		glVertex3f(0.0f,0.0f,0.0f); glVertex3f(0.0f, l1, 0.0f);
		glColor3f(0.0f,0.0f,1.0f);
		glVertex3f(0.0f,0.0f,0.0f); glVertex3f(0.0f, 0.0f, l1);
	glEnd();
	
	m_lighting.Activate();
	glEnable(GL_DEPTH_TEST);	
}

void tgEngine::SetCamera(tgCamera cam){
	m_camera = cam;	
	//m_camera.Print();
}

void tgEngine::UpdateCameraViews(tgCamera cam){
	m_cam[5] = m_cam[4] = m_cam[3] = m_cam[2] = m_cam[1] = m_cam[0] = cam;	
	m_cam[1].Orbit(m_cor, m_cam[1].GetU(), PI);
	m_cam[2].Orbit(m_cor, m_cam[2].GetU(), PI*0.5);
	m_cam[3].Orbit(m_cor, m_cam[3].GetU(),-PI*0.5);
	m_cam[4].Orbit(m_cor, m_cam[4].GetS(), PI*0.5);
	m_cam[5].Orbit(m_cor, m_cam[5].GetS(),-PI*0.5);
}

void tgEngine::SetCenterOfRotation(float x, float y, float z){
	m_cor = tgVector3(x,y,z);
}

void tgEngine::Activate3D(){
	if(m_bfc)	glEnable(GL_CULL_FACE);
	else		glDisable(GL_CULL_FACE);
	m_camera.ApplyTransform();
	m_camera.Activate();
	m_lighting.Activate();
}

void tgEngine::Activate2D(){
	glDisable(GL_CULL_FACE);
	m_cam_ortho.Activate();
	m_lighting.Deactivate();
}

void tgEngine::Swap(){
	m_window->Update();
}

void tgEngine::LoadBackgroundImage(unsigned char* image_data, int width, int height, GLenum format, bool flip){
	
	if(!m_background_image)
		m_background_image = new tgTexture();
	
	m_background_image->Load(image_data, width, height, format);
	m_show_background_image = true;
	m_flip_background_image = flip;
}

void tgEngine::DrawBackgroundImage(){
	if(m_background_image){
		float w = (float)m_width;
		float h = (float)m_height;
		
		Activate2D();
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		
		glDepthMask(0);
		glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
		m_background_image->Bind();
		if(m_flip_background_image){
			glBegin(GL_QUADS);
			  glTexCoord2f(0.0f,1.0f); glVertex3f(0.,		0.,	0.0f);
			  glTexCoord2f(1.0f,1.0f); glVertex3f(w,	0.,	0.0f);
			  glTexCoord2f(1.0f,0.0f); glVertex3f(w,	h, 0.0f);
			  glTexCoord2f(0.0f,0.0f); glVertex3f(0.,		h, 0.0f);
			glEnd();
		}else{
		  glBegin(GL_QUADS);
			  glTexCoord2f(0.0f,0.0f); glVertex3f(0.,		0.,	0.0f);
			  glTexCoord2f(1.0f,0.0f); glVertex3f(w*2,	0.,	0.0f);
			  glTexCoord2f(1.0f,1.0f); glVertex3f(w*2,	h*2, 0.0f);
			  glTexCoord2f(0.0f,1.0f); glVertex3f(0.,		h*2, 0.0f);
		  glEnd();
		}
		glDisable(GL_TEXTURE_2D);
		glDepthMask(1);
		
		if(m_wireframe)
			glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	}
}

void tgEngine::UnloadBackgroundImage(){
	m_show_background_image = false;
	if(m_background_image)
		delete(m_background_image);
	m_background_image = 0;
}

void tgEngine::PrintText3D(std::string text, vec3 pos, int size){
#ifdef USE_FTGL_FONT
	vec2 vPos = m_camera.ToImageSpace(pos);
	PrintText2D(text, vPos, size);
#else
	//printf("[tgEngine::PrintText3D] Warning: ftgl fonts disabled. USE_FTGL_FONT not defined\n");
#endif
}
void tgEngine::PrintText2D(std::string text, vec2 pos, int size){
#ifdef USE_FTGL_FONT
// 	if(!m_font_tf)
// 		return;
// 	Activate2D();
// 	glPushMatrix();
// 		glTranslatef(pos.x, pos.y, 0.0f);
// 		m_font_tf->Render(text.c_str());
// 	glPopMatrix();
// 	Activate3D();
	m_font.Print(text.c_str(), size, pos.x, pos.y);
#else
	//printf("[tgEngine::PrintText2D] Warning: ftgl fonts disabled. USE_FTGL_FONT not defined\n");
#endif
}

tgVector3 tgEngine::Get3DPointFrom2D(int x, int y)
{
	tgVector3 vResult;
	int viewport[4];
	double modelview[16];
	double projection[16];
	float z;
	int y_new;
	double result[3];
	
	m_camera.SetZRange(0.0,1.0);
	
	glGetDoublev(GL_MODELVIEW_MATRIX, &modelview[0] ); //Aktuelle Modelview Matrix in einer Variable ablegen
	glGetDoublev(GL_PROJECTION_MATRIX, &projection[0] ); //Aktuelle Projection[s] Matrix in einer Variable ablegen
	glGetIntegerv(GL_VIEWPORT, &viewport[0] ); // Aktuellen Viewport in einer Variable ablegen
	y_new = viewport[3] - y; // In OpenGL steigt Y von unten (0) nach oben

	// Auslesen des Tiefenpuffers an der Position (X/Y_new)
	glReadPixels(x, y_new, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &z );

	// Errechnen des Punktes, welcher mit den beiden Matrizen multipliziert (X/Y_new/Z) ergibt:
	gluUnProject((double)x, (double)y_new, (double)z, modelview, projection, viewport, &result[0], &result[1], &result[2]); 


	vResult.x = (float)result[0];
	vResult.y = (float)result[1];
	vResult.z = (float)result[2];

	return vResult;
}

bool tgEngine::GetNewPoint(vec3& v){
	if(!p_pressed)
		return false;
	
	p_pressed = false;
	
	tgVector3 tgv = Get3DPointFrom2D(m_mouse_pos[0], m_mouse_pos[1]);
	
	v.x =tgv.x;
	v.y =tgv.y;
	v.z =tgv.z;

	return true;
}

