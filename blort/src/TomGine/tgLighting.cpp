
#include <blort/TomGine/tgLighting.h>

using namespace TomGine;

tgLight::tgLight(){
	ambient  = vec4(0.0, 0.0, 0.0, 1.0);
	diffuse  = vec4(1.0, 1.0, 1.0, 1.0);
	specular = vec4(1.0, 1.0, 1.0, 1.0);
	position = vec4(0.0, 0.0, 1.0, 0.0);
}

void tgLighting::ApplyLight(tgLight light, int index){
	glLightfv(GL_LIGHT0+index, GL_AMBIENT, light.ambient);
	glLightfv(GL_LIGHT0+index, GL_DIFFUSE, light.diffuse);
	glLightfv(GL_LIGHT0+index, GL_SPECULAR, light.specular);
	glLightfv(GL_LIGHT0+index, GL_POSITION, light.position);
	glEnable(GL_LIGHT0+index);
}

void tgLighting::Activate(){
	glEnable(GL_LIGHTING);
}

void tgLighting::Deactivate(){
	glDisable(GL_LIGHTING);
}


