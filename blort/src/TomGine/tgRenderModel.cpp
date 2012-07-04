
#include <blort/TomGine/tgRenderModel.h>
#include <blort/TomGine/tgShapeCreator.h>
#include <GL/gl.h>

using namespace TomGine;

tgRenderModel::tgRenderModel(){
	m_material.Random();
	m_bsmodel = 0;
}

tgRenderModel::tgRenderModel(const tgModel& model){
	m_vertices = model.m_vertices;
	m_faces = model.m_faces;
	m_bsmodel = 0;
	m_material.Random();
}

tgRenderModel::~tgRenderModel(){
	if(m_bsmodel)
		delete(m_bsmodel);
}

void tgRenderModel::ApplyMaterial(){
	glEnable(GL_LIGHTING);
	glMaterialfv(GL_FRONT,GL_AMBIENT,m_material.ambient);
	glMaterialfv(GL_FRONT,GL_DIFFUSE,m_material.diffuse);
	glMaterialfv(GL_FRONT,GL_SPECULAR,m_material.specular);
	glMaterialfv(GL_FRONT,GL_SHININESS,&m_material.shininess);
	glColor4f(m_material.color.x, m_material.color.y, m_material.color.z, m_material.color.w);
}

void tgRenderModel::ApplyColor(){
	glColor4f(m_material.color.x, m_material.color.y, m_material.color.z, m_material.color.w);
}


void tgRenderModel::DrawFaces(){
	DrawFaces(true);
}

void tgRenderModel::DrawFaces(bool lighting){
	if(lighting){
		ApplyMaterial();
	}else{
		glDisable(GL_LIGHTING);
		glColor4f(m_material.color.x, m_material.color.y, m_material.color.z, m_material.color.w);
	}	
	
	m_pose.Activate();
	tgModel::DrawFaces();
	m_pose.Deactivate();
}

void tgRenderModel::DrawNormals(float normal_length){
	m_pose.Activate();
	  tgModel::DrawNormals(normal_length);
	m_pose.Deactivate();
}

void tgRenderModel::DrawBoundingSphere(){
	if(!m_bsmodel){
		m_bsmodel = new tgModel();
		tgShapeCreator creator;
		creator.CreateSphere(*m_bsmodel, m_bs.radius, 2, ICOSAHEDRON);
		if(!m_bsmodel)
			return;
	}else{
	
	  m_pose.Activate();
		  glPushMatrix();
		  glTranslatef(m_bs.center.x, m_bs.center.y, m_bs.center.z);
		  m_bsmodel->DrawFaces();
		  glPopMatrix();
	  m_pose.Deactivate();
	}
}
