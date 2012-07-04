
#include <blort/TomGine/tgMaterial.h>

#ifdef WIN32
#include <GL/glew.h>
#include <GL/gl.h>
#include <GL/glu.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glext.h>
#endif

using namespace TomGine;

tgMaterial::tgMaterial(){
    ambient = vec4(0.5f, 0.5f, 0.5f, 1.0f);
    diffuse = vec4(0.5f, 0.5f, 0.5f, 1.0f);
    specular = vec4(0.5f, 0.5f, 0.5f, 1.0f);
    color = vec4(0.5f, 0.5f, 0.5f, 1.0f);
    shininess = 50.0f;
}

void tgMaterial::Apply(){
    glEnable(GL_LIGHTING);
    glMaterialfv(GL_FRONT,GL_AMBIENT,ambient);
    glMaterialfv(GL_FRONT,GL_DIFFUSE,diffuse);
    glMaterialfv(GL_FRONT,GL_SPECULAR,specular);
    glMaterialfv(GL_FRONT,GL_SHININESS,&shininess);
}

void tgMaterial::Color(float r, float g, float b, float a){
    color = vec4(r,g,b,a);
    ambient = vec4(r,g,b,a) * 0.5f;
    diffuse = vec4(0.2f,0.2f,0.2f,a) + vec4(r,g,b,0.0f) * 0.8f;
    specular = vec4(0.3f,0.3f,0.3f,a);
    shininess = 20.0f;// * float(rand())/RAND_MAX;
}

void tgMaterial::Random(){
    vec4 c;
    c.random();
    Color(c.x, c.y, c.z);
}

