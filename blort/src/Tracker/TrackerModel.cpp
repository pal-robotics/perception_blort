
#include <ros/console.h>
#include <blort/Tracker/TrackerModel.h>
#include <blort/Tracker/Resources.h>

using namespace std;
using namespace Tracking;
using namespace TomGine;

// *** PUBLIC ***

// Constructors
TrackerModel::TrackerModel(){
	m_tex_original = 0;
	m_texture = 0;
	m_textured = false;
	m_bfc = false;
	m_boundingSphereRadius = 0.0;
	m_shadeTexturingID = 0;
	m_dlTexturedFaces = 0;
	m_dlUntexturedFaces = 0;
	m_dlPass = 0;
	m_dlFaces = 0;
	m_dlEdges = 0;
	m_dlNormals = 0;
	
	if((m_shadeTexturingID = g_Resources->AddShader("texturing", "texturing.vert", "texturing.frag")) == -1)
		exit(1);
	m_shadeTexturing = g_Resources->GetShader(m_shadeTexturingID);
}

TrackerModel::~TrackerModel(){	
	releasePassList();
	g_Resources->ReleaseShader(m_shadeTexturingID);
	
	if(m_texture) delete(m_texture);
	if(m_tex_original) delete(m_tex_original);
	
	if(glIsList(m_dlTexturedFaces)) 	glDeleteLists(m_dlTexturedFaces, 1);
	if(glIsList(m_dlUntexturedFaces)) 	glDeleteLists(m_dlUntexturedFaces, 1);
	if(glIsList(m_dlPass)) 				glDeleteLists(m_dlPass, 1);
	if(glIsList(m_dlFaces)) 			glDeleteLists(m_dlFaces, 1);
	if(glIsList(m_dlEdges)) 			glDeleteLists(m_dlEdges, 1);
	if(glIsList(m_dlNormals)) 			glDeleteLists(m_dlNormals, 1);

}

TrackerModel& TrackerModel::operator=(const TrackerModel& m){
	m_vertices = m.m_vertices;
	m_faces = m.m_faces;
	m_lines = m.m_lines;
	m_facepixellist = m.m_facepixellist;
	
	for(unsigned i=0; i<m.m_passlist.size(); i++){
		Pass* p = new(Pass);
		p->f = m.m_passlist[i]->f;
		p->modelviewprojection = m.m_passlist[i]->modelviewprojection;
		p->x = m.m_passlist[i]->x;
		p->y = m.m_passlist[i]->y;
		p->w = m.m_passlist[i]->w;
		p->h = m.m_passlist[i]->h;
		
		// Copy Texture
		if(m.m_passlist[i]->texture){
			int w = m.m_passlist[i]->texture->getWidth();
			int h = m.m_passlist[i]->texture->getHeight();
			int ipw = g_Resources->GetImageProcessor()->getWidth() >> 1;
			int iph = g_Resources->GetImageProcessor()->getHeight() >> 1;
			g_Resources->GetImageProcessor()->render(m.m_passlist[i]->texture, -int(w>>1),-int(h>>1),w,h);
			p->texture->copyTexImage2D(ipw-int(w>>1),iph-int(h>>1),w,h);
		}
		m_passlist.push_back(p);
	}
	
	Update();
	return (*this);
}

TrackerModel::TrackerModel(const TomGine::tgModel& m)
: TomGine::tgModel(m)
{
	m_tex_original = 0;
	m_texture = 0;
	m_textured = false;
	m_bfc = false;
	m_boundingSphereRadius = 0.0;
	m_shadeTexturingID = 0;
	m_dlTexturedFaces = 0;
	m_dlUntexturedFaces = 0;
	m_dlPass = 0;
	m_dlFaces = 0;
	m_dlEdges = 0;
	m_dlNormals = 0;
	
	if((m_shadeTexturingID = g_Resources->AddShader("texturing", "texturing.vert", "texturing.frag")) == -1)
		exit(1);
	m_shadeTexturing = g_Resources->GetShader(m_shadeTexturingID);

	m_lines.clear();
	m_passlist.clear();	
	m_facepixellist.assign(m_faces.size(), 0);
	
	computeEdges();
	Update();
}

void TrackerModel::releasePassList(){
	for(unsigned i=0; i<m_passlist.size(); i++){
		delete(m_passlist[i]);
	}
	m_passlist.clear();
	m_facepixellist.assign(m_faces.size(), 0);
	m_textured = false;
// 	UpdateDisplayLists();
}

// computes, updates
void TrackerModel::computeEdges(){
    int i,j;
    
    // Extract edges from faces
    for(i=0; i<(int)m_faces.size(); i++){
        for(j=0; j<(int)m_faces[i].v.size(); j++){
        	tgLine e;
            e.start = m_vertices[m_faces[i].v[j]].pos;
            e.end = m_vertices[m_faces[i].v[(j+1)%m_faces[i].v.size()]].pos;
            if(!isRedundant(&e)){
                //ROS_DEBUG("Edge: %d %d\n", m_edgelist[k].start, m_edgelist[k].end);
            	m_lines.push_back(e);
            }
        }
    }
}

void TrackerModel::computeBoundingSphere(){
	m_boundingSphereRadius = 0.0;
	
	float r = 0.0;
	vec3 v;
	
	for(unsigned i=0; i<m_vertices.size(); i++){
		v = m_vertices[i].pos;
		r = sqrt( pow(v.x,2) + pow(v.y,2) + pow(v.z,2) );
		if(r>m_boundingSphereRadius)
			m_boundingSphereRadius = r;
	}
}

void TrackerModel::Update(){
	unsigned i,j;
	if(m_facepixellist.size() != m_faces.size())
		m_facepixellist.assign(m_faces.size(), 0);
				
	for(i=0; i<m_passlist.size(); i++){
		for(j=0; j<m_passlist[i]->f.size(); j++){
			m_facepixellist[m_passlist[i]->f[j]] = 1;
		}
	}
	
	computeBoundingSphere();
	
// 	UpdateDisplayLists();
}

// draws, prints
void TrackerModel::Print() const{
	tgModel::Print();
        for(unsigned i=0; i<m_lines.size(); i++){
                ROS_DEBUG("Edge %i: %f %f %f,  %f %f %f\n", i,
				m_lines[i].start.x, m_lines[i].start.y, m_lines[i].start.z,
				m_lines[i].end.x, m_lines[i].end.y, m_lines[i].end.z);
	}
}

void TrackerModel::drawNormals(){
	genListNormals(0.01f);
// 	glCallList(m_dlNormals);
}

void TrackerModel::drawTexturedFaces(){
	if(!m_passlist.empty()){
		genListTexturedFaces();
// 		glCallList(m_dlTexturedFaces);
	}		
}

void TrackerModel::drawUntexturedFaces(){
	genListUntexturedFaces();
// 	glCallList(m_dlUntexturedFaces);
}

void TrackerModel::drawPass(bool colorful){
 	genListPass(colorful);
//  	glCallList(m_dlPass);
}

void TrackerModel::drawFaces(bool colorful){
	if(m_texture)	m_texture->bind();
	
	genListFaces(colorful);
// 	glCallList(m_dlFaces);

	if(m_texture) glDisable(GL_TEXTURE_2D);	
}

void TrackerModel::drawEdges(){
	genListEdges();
// 	glCallList(m_dlEdges);
}

void TrackerModel::drawFace(int i){
	int j;
	tgFace* f;
	
	if(m_texture){
		glActiveTexture(GL_TEXTURE0);
		glEnable(GL_TEXTURE_2D);
		m_texture->bind();
	}

	f = &m_faces[i];
	if((int)f->v.size() == 3)
		glBegin(GL_TRIANGLES);
	else if(f->v.size() == 4)
		glBegin(GL_QUADS);
	else
                ROS_DEBUG("[TrackerModel::drawFaces] Warning unsupported face structure");
	
	for(j=0; j<(int)f->v.size(); j++){
		glTexCoord2f(m_vertices[f->v[j]].texCoord.x, m_vertices[f->v[j]].texCoord.y);
		glNormal3f(m_vertices[f->v[j]].normal.x, m_vertices[f->v[j]].normal.y, m_vertices[f->v[j]].normal.z);
		glVertex3f(m_vertices[f->v[j]].pos.x, m_vertices[f->v[j]].pos.y, m_vertices[f->v[j]].pos.z);
	}
		
	glEnd();
	
	if(m_texture){
		glDisable(GL_TEXTURE_2D);
	}
}

void TrackerModel::drawCoordinates(){
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_CULL_FACE);
	
	float l1 = 0.06f;
	float l2 = 0.02f;
	float b1 = 0.001f;
	float b2 = 0.003f;
	
	// X - Axis
	glPushMatrix();
		glColor3f(1.0,0.0,0.0);
		glBegin(GL_TRIANGLE_FAN);
			glVertex3f(l1,		0.0,	 b1);
			glVertex3f(0.0,		0.0,	 b1);
			glVertex3f(0.0,		0.0,	-b1);
			glVertex3f(l1,		0.0,	-b1);
			glVertex3f(l1,		0.0,	-b1-b2);
			glVertex3f(l1+l2,	0.0,	0.0);
			glVertex3f(l1,		0.0,	 b1+b2);
		glEnd();
	glPopMatrix();
		
	// Y - Axis
	glPushMatrix();
		glColor3f(0.0,1.0,0.0);
		glRotatef(90, 0.0, 0.0, 1.0);
		glBegin(GL_TRIANGLE_FAN);
			glVertex3f(l1,		0.0,	 b1);
			glVertex3f(0.0,		0.0,	 b1);
			glVertex3f(0.0,		0.0,	-b1);
			glVertex3f(l1,		0.0,	-b1);
			glVertex3f(l1,		0.0,	-b1-b2);
			glVertex3f(l1+l2,	0.0,	0.0);
			glVertex3f(l1,		0.0,	 b1+b2);
		glEnd();
	glPopMatrix();
	
	// Z - Axis
	glPushMatrix();
		glColor3f(0.0,0.0,1.0);
		glRotatef(-90, 0.0, 1.0, 0.0);
		glBegin(GL_TRIANGLE_FAN);
			glVertex3f(l1,		0.0,	 b1);
			glVertex3f(0.0,		0.0,	 b1);
			glVertex3f(0.0,		0.0,	-b1);
			glVertex3f(l1,		0.0,	-b1);
			glVertex3f(l1,		0.0,	-b1-b2);
			glVertex3f(l1+l2,	0.0,	0.0);
			glVertex3f(l1,		0.0,	 b1+b2);
		glEnd();
	glPopMatrix();
}

// counts pixels of each face
// if pixels of face are > than in any previouse view
//   set update flag = true
vector<unsigned> TrackerModel::getFaceUpdateList(tgPose& p_max, vec3 view, float minTexGrabAngle, bool use_num_pixels){
	int i, n;
	vector<unsigned> faceUpdateList;
	float alpha;
	vec3 vT;
	mat3 mR;
	p_max.GetPose(mR, vT);
	view.normalize();
	
	p_max.Activate();
	
	// count pixels for each face and choose if its texture has to be updated

	unsigned int* queryPixels;		// Occlussion query for counting pixels
	queryPixels = (unsigned int*)malloc( sizeof(unsigned int) * m_faces.size() );
	glGenQueriesARB(m_faces.size(), queryPixels);
	for(i=0; i<(int)m_faces.size(); i++){
		glBeginQueryARB(GL_SAMPLES_PASSED_ARB, queryPixels[i]);

		drawFace(i);

		glEndQueryARB(GL_SAMPLES_PASSED_ARB);			
	}

	for(i=0; i<(int)m_faces.size(); i++){

		glGetQueryObjectivARB(queryPixels[i], GL_QUERY_RESULT_ARB, &n);
	
		vec3 vn = mR * m_faces[i].normal;
		alpha = acos(vn*view);
		
		if(alpha>minTexGrabAngle){
			if(use_num_pixels){
				if(m_facepixellist[i]==0){
					faceUpdateList.push_back(i);
					m_facepixellist[i] = n;
				}
			}else{
				faceUpdateList.push_back(i);
				m_facepixellist[i] = n;
			}
		}

	}
	p_max.Deactivate();
	
	glDeleteQueriesARB(m_faces.size(), queryPixels);
	free(queryPixels);

	return faceUpdateList;
}

void TrackerModel::getBoundingBox2D( int width, int height, TomGine::tgPose& p_max, TomGine::tgCamera* m_cam,
									 int &minX, int &maxX, int &minY, int &maxY)
{
	// Extract transformation matrix (from object to image space)
	mat4 modelview, projection, modelviewprojection;
	mat3 rot;
	vec3 pos;
	p_max.GetPose(rot,pos);
	mat4 matModel = mat4(rot);
	matModel[12]=pos.x; matModel[13]=pos.y; matModel[14]=pos.z;
	mat4 matView = m_cam->GetExtrinsic();
	projection = m_cam->GetIntrinsic();
	modelview = matView * matModel;
	modelviewprojection = projection * modelview;
	
	// calculate bounding rectangle
	vec4 texCoords;
	vec3 p;
	float x_min=(float)width;
	float y_min=(float)height;
	float x_max=0.0;
	float y_max=0.0;
	float x,y;
	for(unsigned j=0; j<m_vertices.size(); j++){
		p = m_vertices[j].pos;
		texCoords = modelviewprojection * vec4(p.x, p.y, p.z, 1.0);
		x = (texCoords.x / texCoords.w + 1.0f) * 0.5f;
		y = (texCoords.y / texCoords.w + 1.0f) * 0.5f;
		if(x<x_min)	x_min = x;
		if(y<y_min)	y_min = y;
		if(x>x_max) x_max = x;
		if(y>y_max)	y_max = y;
	}
	
	// Calculate bounding rectangle in pixels
	minX = (int)round(x_min * width);
	maxX = (int)round(x_max * width);
	minY = (int)round(y_min * height);
	maxY = (int)round(y_max * height);
}

void TrackerModel::textureFromImage(Texture* image,
									int width, int height,
									tgPose& p_max,
									vec3 view,
									float minTexGrabAngle,
									std::vector<unsigned> faceUpdateList,
									std::vector<tgVertex> &vertices,
									TomGine::tgCamera* m_cam)
{
	unsigned i,j;
	int k;
	vec4 texcoords_model;
	vec4 vertex;
	//double mv[16];
	//double pj[16];
	mat4 modelview, projection, modelviewprojection;

	// add new rendering pass
	Pass* newpass = new Pass;
	
	// query modelview and projection matrix
// 	p_max.activate();
// 	glGetDoublev(GL_MODELVIEW_MATRIX, mv);		// TODO to inaccurate, replace with modelview of p_max and projection of camera
// 	glGetDoublev(GL_PROJECTION_MATRIX, pj);
// 	p_max.deactivate();
// 	for(i=0; i<16; i++){
// 		modelview.mat[i] = (float)mv[i];
// 		projection.mat[i] = (float)pj[i];
// 	}
// 	newpass->modelviewprojection = projection * modelview;
	mat3 rot;
	vec3 pos;
	p_max.GetPose(rot,pos);
	mat4 matModel = mat4(rot);
	matModel[12]=pos.x; matModel[13]=pos.y; matModel[14]=pos.z;
	mat4 matView = m_cam->GetExtrinsic();
	projection = m_cam->GetIntrinsic();
	modelview = matView * matModel;
	modelviewprojection = projection * modelview;
	newpass->modelviewprojection = modelviewprojection;
	
	
	// calculate bounding rectangle
	vec4 texCoords;
	vec3 p;
	tgVertex v;
	float x_min=(float)width;
	float y_min=(float)height;
	float x_max=0.0;
	float y_max=0.0;
	float x,y;
	float w2 = float(width>>1);
	float h2 = float(height>>1);
	for(i=0; i<faceUpdateList.size(); i++){
		k = faceUpdateList[i];
		for(j=0; j<m_faces[k].v.size(); j++){
			p = m_vertices[m_faces[k].v[j]].pos;
			texCoords = modelviewprojection * vec4(p.x, p.y, p.z, 1.0);
			x = (texCoords.x / texCoords.w + 1.0f) * 0.5f;
			y = (texCoords.y / texCoords.w + 1.0f) * 0.5f;
			m_vertices[m_faces[k].v[j]].texCoord.x = x;
			m_vertices[m_faces[k].v[j]].texCoord.y = y;
			if(x<x_min)	x_min = x;
			if(y<y_min)	y_min = y;
			if(x>x_max) x_max = x;
			if(y>y_max)	y_max = y;
		}
	}
	
	// Store bounding box
	newpass->x 	= x_min;
	newpass->w 	= x_max-x_min;
	newpass->y 	= y_min;
	newpass->h 	= y_max-y_min;
	
	// Calculate bounding rectangle in pixels
	x_min = (x_min * width);
	x_max = (x_max * width);
	y_min = (y_min * height);
	y_max = (y_max * height);

	v.pos.x = x_min - w2;
	v.pos.y = y_min - h2;
	vertices.push_back(v);
	v.pos.x = x_max - w2;
	v.pos.y = y_min - h2;
	vertices.push_back(v);
	v.pos.x = x_max - w2;
	v.pos.y = y_max - h2;
	vertices.push_back(v);
	v.pos.x = x_min - w2;
	v.pos.y = y_max - h2;
	vertices.push_back(v);
	
	// store texture
	newpass->texture->copyFromTexture(image, (int)x_min, (int)y_min, unsigned(x_max-x_min), unsigned(y_max-y_min));
	
	// add faces to pass
	newpass->f = faceUpdateList;
	
	m_passlist.push_back(newpass);
			
	// clean up passes
        std::vector<unsigned int> texturedfaces;
	int m=0;
	for(m=(m_passlist.size()-1); m>=0; m--){		// parse through passlist topdown
		Pass* p = m_passlist[m];					// current pass
		bool destroy = true;
		
		for(j=0; j<p->f.size(); j++){				// for each face of pass
			bool face_allready_in_use = false;
			for(k=0; k<(int)texturedfaces.size(); k++){
				if(p->f[j] == texturedfaces[k])			// compare with each face in usedfaces
					face_allready_in_use = true;
			}
			if(!face_allready_in_use){
				texturedfaces.push_back(p->f[j]);
				destroy=false;
			}
		}
		
		if(destroy){
			delete(p);
			m_passlist.erase(m_passlist.begin()+m);
		}
	}
	
	if(!m_passlist.empty())
		m_textured = true;
	
// 	UpdateDisplayLists();
}

void TrackerModel::useTexCoords(bool useTC){

	m_shadeTexturing->bind();
	m_shadeTexturing->setUniform("useTexCoords",useTC);
	m_shadeTexturing->unbind();

}

void TrackerModel::unwarpTexturesBox_hacky(const char* name){

	char charbuffer[8];
	Texture tex;
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);

	for(unsigned p=0; p<m_passlist.size(); p++){

		if(m_passlist[p]->f.size() != 1){
                        ROS_DEBUG("[TrackerModel::unwarpTexturesBox_hacky] Warning no more than one face per pass allowed\n");
			return;
		}

		tgFace* f = &m_faces[m_passlist[p]->f[0]];
		if(f->v.size()!=4){
                        ROS_DEBUG("[TrackerModel::unwarpTexturesBox_hacky] Warning only quad faces allowed\n");
			return;
		}

		ImageProcessor *ip = g_Resources->GetImageProcessor();
		ip->setCamOrtho();
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		float x = m_passlist[p]->h * ip->getHeight() * 0.75;
		float y = m_passlist[p]->w * ip->getWidth() * 0.75;
		float z = 0.0;
		float nz = 1.0;

		glEnable(GL_TEXTURE_2D);

		m_passlist[p]->texture->bind();

		glBegin(GL_QUADS);
			unsigned j=0;
			glTexCoord2f(m_vertices[f->v[j]].texCoord.x, m_vertices[f->v[j]].texCoord.y);
			glNormal3f(0.0, 0.0, nz);
			glVertex3f(-x, -y, z);

			j=1;
			glTexCoord2f(m_vertices[f->v[j]].texCoord.x, m_vertices[f->v[j]].texCoord.y);
			glNormal3f(0.0, 0.0, nz);
			glVertex3f( x, -y, z);

			j=2;
			glTexCoord2f(m_vertices[f->v[j]].texCoord.x, m_vertices[f->v[j]].texCoord.y);
			glNormal3f(0.0, 0.0, nz);
			glVertex3f( x, y, z);

			j=3;
			glTexCoord2f(m_vertices[f->v[j]].texCoord.x, m_vertices[f->v[j]].texCoord.y);
			glNormal3f(0.0, 0.0, nz);
			glVertex3f(-x, y, z);
		glEnd();

		int ix = (ip->getWidth() >> 1) - (int)x;
		int iy = (ip->getHeight() >> 1) - (int)y;

		tex.copyTexImage2D(ix, iy, int(x)<<1, int(y)<<1);

		std::string texname = std::string(name);
		texname.append("-unwrap-");
		sprintf(charbuffer, "%.5u", p);
		texname.append(charbuffer);
		texname.append(".jpg");
		tex.save(texname.c_str());

	}

}

// *** PROTECTED ***

// Tests redundancy of edge
bool TrackerModel::isRedundant(TomGine::tgLine* e1){
	tgLine* e2;
	vec3 vs, ve;
	
	for(int i=0; i<(int)m_lines.size(); i++){
		e2 = &m_lines[i];
		
		// Get Vector between start-start and end-end points of edges
		vs = e1->start - e2->start;
		ve = e1->end - e2->end;
//		vs = vec3(	m_vertices[e1->start].x - m_vertices[e2->start].pos.x,
//					m_vertices[e1->start].pos.y - m_vertices[e2->start].pos.y,
//					m_vertices[e1->start].pos.z - m_vertices[e2->start].pos.z);
//		ve = vec3(	m_vertices[e1->end].pos.x - m_vertices[e2->end].pos.x,
//					m_vertices[e1->end].pos.y - m_vertices[e2->end].pos.y,
//					m_vertices[e1->end].pos.z - m_vertices[e2->end].pos.z);
		// if sum of length between vertices is insignificant then redundancy is detected
		if(vs.length() + ve.length() < 0.01){
                        //ROS_DEBUG("Redundant edge detected: %d %d\n", e1->start, e1->end);
			return true;
		}
		
		// Get Vector between start-end and end-start points of edges
		vs = e1->start - e2->end;
		ve = e1->end - e2->start;
//		vs = vec3(	m_vertices[e1->start].pos.x - m_vertices[e2->end].pos.x,
//					m_vertices[e1->start].pos.y - m_vertices[e2->end].pos.y,
//					m_vertices[e1->start].pos.z - m_vertices[e2->end].pos.z);
//		ve = vec3(	m_vertices[e1->end].pos.x - m_vertices[e2->start].pos.x,
//					m_vertices[e1->end].pos.y - m_vertices[e2->start].pos.y,
//					m_vertices[e1->end].pos.z - m_vertices[e2->start].pos.z);
		// if sum of length between vertices is insignificant then redundancy is detected
		if(vs.length() + ve.length() < 0.01){
                        //ROS_DEBUG("Redundant edge detected: %d %d\n", e1->start, e1->end);
			return true;
		}
						
	}
	return false;
}

// Generate all display lists
void TrackerModel::UpdateDisplayLists(){

	if(glIsList(m_dlTexturedFaces)) 	glDeleteLists(m_dlTexturedFaces, 1);
	if(glIsList(m_dlUntexturedFaces)) glDeleteLists(m_dlUntexturedFaces, 1);
	if(glIsList(m_dlPass)) 						glDeleteLists(m_dlPass, 1);
	if(glIsList(m_dlFaces)) 					glDeleteLists(m_dlFaces, 1);
	if(glIsList(m_dlEdges)) 					glDeleteLists(m_dlEdges, 1);
	if(glIsList(m_dlNormals)) 				glDeleteLists(m_dlNormals, 1);

	m_dlTexturedFaces = glGenLists(1);
	m_dlUntexturedFaces = glGenLists(1);
	m_dlPass = glGenLists(1);
	m_dlFaces = glGenLists(1);
	m_dlEdges = glGenLists(1);
	m_dlNormals = glGenLists(1);

	if(!m_passlist.empty()){
		glNewList(m_dlTexturedFaces, GL_COMPILE);
			genListTexturedFaces();
		glEndList();
		
		glNewList(m_dlPass, GL_COMPILE);
			genListPass();
		glEndList();
	}else{
		glNewList(m_dlPass, GL_COMPILE);
			genListFaces();
		glEndList();
	}
	
	glNewList(m_dlUntexturedFaces, GL_COMPILE);
		genListUntexturedFaces();
	glEndList();
	
	glNewList(m_dlFaces, GL_COMPILE);
		genListFaces();
	glEndList();
	
	glNewList(m_dlEdges, GL_COMPILE);
		genListEdges();
	glEndList();
	
	glNewList(m_dlNormals, GL_COMPILE);
		genListNormals(0.01f);
	glEndList();	
}

// Display List generators
void TrackerModel::genListTexturedFaces(){		// draw only textured faces
		
	int p,i,j;
	tgFace* f;
	
	if(m_bfc) glEnable(GL_CULL_FACE);
	else glDisable(GL_CULL_FACE);
	
	for(p=0; p<(int)m_passlist.size(); p++){
		// parse through faces of pass
		for(i=0; i<(int)m_passlist[p]->f.size(); i++){
			f = &m_faces[m_passlist[p]->f[i]];
			
			if(f->v.size() == 3)
				glBegin(GL_TRIANGLES);
			else if(f->v.size() == 4)
				glBegin(GL_QUADS);
			else
                                ROS_DEBUG("[TrackerModel::drawFaces] Warning unsupported face structure");
				
			for(j=0; j<(int)f->v.size(); j++){
				glTexCoord2f(m_vertices[f->v[j]].texCoord.x, m_vertices[f->v[j]].texCoord.y);
				glNormal3f(m_vertices[f->v[j]].normal.x, m_vertices[f->v[j]].normal.y, m_vertices[f->v[j]].normal.z);
				glVertex3f(m_vertices[f->v[j]].pos.x, m_vertices[f->v[j]].pos.y, m_vertices[f->v[j]].pos.z);
			}
			glEnd();			
		}//for i
	}//for p
}

void TrackerModel::genListUntexturedFaces(){		// draw only untextured faces
	int i,j;
	tgFace* f;
	
	if(m_bfc) glEnable(GL_CULL_FACE);
	else glDisable(GL_CULL_FACE);
	
	for(i=0; i<(int)m_faces.size(); i++){
		
		f = &m_faces[i];
		if(m_facepixellist[i] == 0){
		
			if(f->v.size() == 3)
				glBegin(GL_TRIANGLES);
			else if(f->v.size() == 4)
				glBegin(GL_QUADS);
			else
                                ROS_DEBUG("[TrackerModel::drawFaces] Warning unsupported face structure");
				
			for(j=0; j<(int)f->v.size(); j++){
				glTexCoord2f(m_vertices[f->v[j]].texCoord.x, m_vertices[f->v[j]].texCoord.y);
				glNormal3f(m_vertices[f->v[j]].normal.x, m_vertices[f->v[j]].normal.y, m_vertices[f->v[j]].normal.z);
				glVertex3f(m_vertices[f->v[j]].pos.x, m_vertices[f->v[j]].pos.y, m_vertices[f->v[j]].pos.z);
			}
			
			glEnd();			
		}//if
	}	//for
}

void TrackerModel::genListPass(bool colorful){		// draw faces using passlist and shader for texturing (modelviewmatrix)
	int p,i,j;
	tgFace* f;
	vec2 texCoords;
	
	if(m_bfc) glEnable(GL_CULL_FACE);
	else glDisable(GL_CULL_FACE);
	
	glActiveTexture(GL_TEXTURE0);
	glEnable(GL_TEXTURE_2D);
	glColor3f(1.0,1.0,1.0);

	m_shadeTexturing->bind();
	
	// Draw render passes (textured)
	for(p=0; p<(int)m_passlist.size(); p++){
		
		// bind texture of pass
		m_passlist[p]->texture->bind();
		// set modelview matrix for texture
// 		ROS_DEBUG("x,y,w,h: %f %f %f %f\n", m_passlist[p]->x, m_passlist[p]->y, m_passlist[p]->w, m_passlist[p]->h);
// 		ROS_DEBUG("face %d: %d %d %d %d\n", m_passlist[p]->f[0],
// 				m_faces[m_passlist[p]->f[0]].v[0], 
// 				m_faces[m_passlist[p]->f[0]].v[1],
// 				m_faces[m_passlist[p]->f[0]].v[2], 
// 				m_faces[m_passlist[p]->f[0]].v[3]);
// 		ROS_DEBUG("modelview: %f %f %f %f\n", m_passlist[p]->modelviewprojection.mat[0], m_passlist[p]->modelviewprojection.mat[1], m_passlist[p]->modelviewprojection.mat[2], m_passlist[p]->modelviewprojection.mat[3]);

		m_shadeTexturing->setUniform("modelviewprojection", m_passlist[p]->modelviewprojection, GL_FALSE);
		m_shadeTexturing->setUniform("x", m_passlist[p]->x);
		m_shadeTexturing->setUniform("y", m_passlist[p]->y);
		m_shadeTexturing->setUniform("w", m_passlist[p]->w);
		m_shadeTexturing->setUniform("h", m_passlist[p]->h);
		
		// parse through faces of pass
		for(i=0; i<(int)m_passlist[p]->f.size(); i++){
			f = &m_faces[m_passlist[p]->f[i]];
			
			if(f->v.size() == 3)
				glBegin(GL_TRIANGLES);
			else if(f->v.size() == 4)
				glBegin(GL_QUADS);
			else
                                ROS_DEBUG("[TrackerModel::drawFaces] Warning unsupported face structure");
				
				for(j=0; j<(int)f->v.size(); j++){
					
// 					vec4 v = vec4(m_vertices[f->v[j]].pos.x, m_vertices[f->v[j]].pos.y, m_vertices[f->v[j]].pos.z, 1.0);
// 					v = m_passlist[p]->modelviewprojection * v;
// 					texCoords.x = (v.x/v.w + 1.0) * 0.5;
// 					texCoords.y = (v.y/v.w + 1.0) * 0.5;
// 					glTexCoord2f(texCoords.x, texCoords.y);

					glTexCoord2fv(m_vertices[f->v[j]].texCoord);
					glNormal3fv(m_vertices[f->v[j]].normal);
					glVertex3fv(m_vertices[f->v[j]].pos);
				}
				
			glEnd();
		}//for i
	}//for p
		
	glDisable(GL_TEXTURE_2D);
	m_shadeTexturing->unbind();
	
	if(colorful) glDisable(GL_LIGHTING);
	vec3 color;
	
	// Draw remaining faces for contours
	for(i=0; i<(int)m_faces.size(); i++){
		
		if(m_facepixellist[i] == 0){
				
			if(m_faces[i].v.size() == 3)
				glBegin(GL_TRIANGLES);
			else if(m_faces[i].v.size() == 4)
				glBegin(GL_QUADS);
			else
                                ROS_DEBUG("[TrackerModel::drawFaces] Warning unsupported face structure");
			
			color.random();
			for(j=0; j<(int)m_faces[i].v.size(); j++){				
				glNormal3fv(m_vertices[m_faces[i].v[j]].normal);
				if(colorful) glColor3fv(color);
				glVertex3fv(m_vertices[m_faces[i].v[j]].pos);
// 				ROS_DEBUG("%f %f %f\n", m_vertices[m_faces[i].v[j]].normal.x, m_vertices[m_faces[i].v[j]].normal.y, m_vertices[m_faces[i].v[j]].normal.z);
			}
			
			glEnd();
		}//if
	}//for i
}

void TrackerModel::genListFaces(bool colorful){		// draw all faces of model
	int i,j;
	tgFace* f;
	
	if(m_bfc) glEnable(GL_CULL_FACE);
	else glDisable(GL_CULL_FACE);

	for(i=0; i<(int)m_faces.size(); i++){
		f = &m_faces[i];
		
		if((int)f->v.size() == 3)
			glBegin(GL_TRIANGLES);
		else if((int)f->v.size() == 4)
			glBegin(GL_QUADS);
		else
                        ROS_DEBUG("[TrackerModel::drawFaces] Warning unsupported face structure");
			
			for(j=0; j<(int)f->v.size(); j++){
				glTexCoord2f(m_vertices[f->v[j]].texCoord.x, m_vertices[f->v[j]].texCoord.y);
				glNormal3f(m_vertices[f->v[j]].normal.x, m_vertices[f->v[j]].normal.y, m_vertices[f->v[j]].normal.z);
				glVertex3f(m_vertices[f->v[j]].pos.x, m_vertices[f->v[j]].pos.y, m_vertices[f->v[j]].pos.z);
			}
			
		glEnd();
	}
}

void TrackerModel::genListEdges(){		// draw all edges of model
	mat4 mv;
	mat3 rot;
	vec3 v_cam_object;
	float s = -0.001f;	// = 1mm
	
	glGetFloatv(GL_MODELVIEW_MATRIX, mv);
	
        //ROS_DEBUG("%f %f %f %f\n", mv[0], mv[4], mv[8], mv[12]);
        //ROS_DEBUG("%f %f %f %f\n", mv[1], mv[5], mv[9], mv[13]);
        //ROS_DEBUG("%f %f %f %f\n", mv[2], mv[6], mv[10], mv[14]);
        //ROS_DEBUG("%f %f %f %f\n\n", mv[3], mv[7], mv[11], mv[15]);
	
	rot[0] = mv[0]; rot[1] = mv[4]; rot[2] = mv[8];
	rot[3] = mv[1]; rot[4] = mv[5]; rot[5] = mv[9];
	rot[6] = mv[2]; rot[7] = mv[6]; rot[8] = mv[10];
	
	v_cam_object[0] = mv[12];
	v_cam_object[1] = mv[13];
	v_cam_object[2] = mv[14];
	
	v_cam_object = rot * v_cam_object * s;
	
	glPushMatrix();
		// draw edges slightly closer to camera to avoid edge-flickering
		glTranslatef(v_cam_object[0], v_cam_object[1], v_cam_object[2]);

		glBegin(GL_LINES);
			for(int i=0; i<(int)m_lines.size(); i++){
				glNormal3f( m_lines[i].end.x - m_lines[i].start.x,
							m_lines[i].end.y - m_lines[i].start.y,
							m_lines[i].end.z - m_lines[i].start.z );
				glVertex3f(m_lines[i].start.x, m_lines[i].start.y, m_lines[i].start.z);
				glVertex3f(m_lines[i].end.x, m_lines[i].end.y, m_lines[i].end.z);
			}
		glEnd();

    glPopMatrix();
}

void TrackerModel::genListNormals(float normal_length){	// draw normals
	unsigned i,j;
	tgFace* f;
	
	glDisable(GL_TEXTURE_2D);
	glColor3f(0.0, 0.0, 1.0);
	
	glBegin(GL_LINES);
	for(i=0; i<m_faces.size(); i++){
		f = &m_faces[i];
		for(j=0; j<f->v.size(); j++){
			glVertex3f( m_vertices[f->v[j]].pos.x,
									m_vertices[f->v[j]].pos.y,
									m_vertices[f->v[j]].pos.z );
			glVertex3f( m_vertices[f->v[j]].pos.x + m_vertices[f->v[j]].normal.x * normal_length,
									m_vertices[f->v[j]].pos.y + m_vertices[f->v[j]].normal.y * normal_length,
									m_vertices[f->v[j]].pos.z + m_vertices[f->v[j]].normal.z * normal_length );
		}
	}
	glEnd();
	
	glColor3f(1.0, 1.0, 1.0);
}



