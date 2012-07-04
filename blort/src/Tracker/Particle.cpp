
#include <blort/Tracker/Particle.h>

using namespace Tracking;
using namespace TomGine;

Particle::Particle(float val){
	t.x = val;
	t.y = val;
	t.z = val;
	r.x = val;
	r.y = val;
	r.z = val;
	
	z = val;
	
	w = val;
	c = val;
	
	q = tgQuaternion();
}

Particle::Particle(	vec3 t,
					vec3 r,
					float z,
					float w, float c,
					TomGine::tgQuaternion q)
{
	this->t = t;
	this->r = r;
	this->z = z;
	
	this->w = w;
	this->c = c;
	
	this->q = q;
}

Particle::Particle(const Particle& p2){
	t.x = p2.t.x;
	t.y = p2.t.y;
	t.z = p2.t.z;
	r.x = p2.r.x;
	r.y = p2.r.y;
	r.z = p2.r.z;

	z = p2.z;
	
	userData = p2.userData;
	
	w = p2.w;
	c = p2.c;
	
	q = p2.q;
}

Particle::Particle(const tgPose& p2){
	t = p2.t;
	r.x = 0.0;
	r.y = 0.0;
	r.z = 0.0;
	
	z = 0.0;	
	
	w = 0.0;
	c = 0.0;
	
	q = p2.q;
}

Particle& Particle::operator=(const Particle& p2){
	t.x = p2.t.x;
	t.y = p2.t.y;
	t.z = p2.t.z;
	r.x = p2.r.x;
	r.y = p2.r.y;
	r.z = p2.r.z;
	z = p2.z;
	
	userData = p2.userData;
	
	w = p2.w;
	c = p2.c;
	
	q = p2.q;
	
	return *this;
}

Particle& Particle::operator=(const tgPose& p2){
	t = p2.t;
	r.x = 0.0;
	r.y = 0.0;
	r.z = 0.0;
	
	z = 0.0;	
	
	w = 0.0;
	c = 0.0;
	
	q = p2.q;

	return *this;
}

Particle Particle::operator*(const float &f) const{
	Particle p;
	p.r  = r  * f;
	p.t  = t  * f;
	p.z  = z  * f;
	
	return p;
}

Particle Particle::operator+(const Particle &p) const{
	Particle res;
	res.t = t + p.t;
	res.q = q + p.q;
	res.r = r + p.r;
	res.z = z + p.z;
	res.w = w + p.w;
	res.c = c + p.c;
	return res;
}

Particle Particle::operator-(const Particle &p) const{
	Particle res;
	res.t = t - p.t;
	res.q = q - p.q;
	res.r = r - p.r;
	res.z = z - p.z;
	res.w = w - p.w;
	res.c = c - p.c;
	return res;
}
