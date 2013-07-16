//CPP:
//			Title:			class tgVector3
//			File:				tgVector3.cpp
//
//			Function:		3D Vector with all necessary operations 
//
//			Author:			Thomas Moerwald
//			Date:				10.01.2007
// ----------------------------------------------------------------------------
#include <blort/TomGine/tgVector3.h>
#include <blort/TomGine/tgMatrix3.h>

using namespace TomGine;

tgVector3::tgVector3()
:x(0), y(0), z(0)
{
//	tgVector3(0.0f);
}

tgVector3::tgVector3(float all)
:x(all), y(all), z(all)
{
	//tgVector3(all,all,all);
	
}

tgVector3::tgVector3(float xn, float yn, float zn)
{
	x = xn; y = yn; z = zn;
}

tgVector3 tgVector3::operator=(tgVector3 v){
	x = v.x;
	y = v.y;
	z = v.z;
	return (*this);
}

tgVector3 tgVector3::operator+(tgVector3 v)
{
	return tgVector3(x+v.x, y+v.y, z+v.z);
}

tgVector3 tgVector3::operator-(tgVector3 v)
{
	return tgVector3(x-v.x, y-v.y, z-v.z);
}

tgVector3 tgVector3::operator*(tgVector3 v)
{
	return tgVector3(x*v.x, y*v.y, z*v.z);
}

tgVector3 tgVector3::operator*(float m)
{
	return tgVector3(x*m, y*m, z*m);
}

tgVector3 tgVector3::operator/(float m)
{
	return tgVector3(x/m, y/m, z/m);
}

tgVector3 tgVector3::cross(tgVector3 v)
{
	return tgVector3(	y*v.z - z*v.y, 
					z*v.x - x*v.z,
					x*v.y - y*v.x);
}

tgVector3 tgVector3::cross(tgVector3 v1, tgVector3 v2)
{
	return tgVector3(	v1.y*v2.z - v1.z*v2.y, 
					v1.z*v2.x - v1.x*v2.z,
					v1.x*v2.y - v1.y*v2.x);
}

float tgVector3::dot(tgVector3 v)
{
	return (x*v.x + y*v.y + z*v.z);
}

void tgVector3::normalize()
{
	float s = sqrt(x*x + y*y + z*z);
	if(s != 0.0f)
	{
		x /= s;
		y /= s;
		z /= s;
	}
}

float tgVector3::length()
{
	return sqrt(x*x + y*y + z*z);
}

void tgVector3::setLength(float l)
{
	normalize();
	x *= l;
	y *= l;
	z *= l;	
}

void tgVector3::rotateX(float fAngle)
{
	float tz = z;
	float ty = y;
	y = ty * cos(fAngle) - tz * sin(fAngle);
	z = ty * sin(fAngle) + tz * cos(fAngle);
}

void tgVector3::rotateY(float fAngle)
{	
	float tz = z;
	float tx = x;
	z = tz * cos(fAngle) - tx * sin(fAngle);
	x = tz * sin(fAngle) + tx * cos(fAngle);
}

void tgVector3::rotateZ(float fAngle)
{
	float tx = x;
	float ty = y;
	x = tx * cos(fAngle) - ty * sin(fAngle);
	y = tx * sin(fAngle) + ty * cos(fAngle);
}

void tgVector3::rotate(float alpha, tgVector3 r) {
	
	tgVector3 v,s,t,n;
	tgMatrix3 M,Mt,Rx,X;
	
	if(alpha != 0){

		r.normalize();
		s = r.cross(tgVector3(1,0,0));
		
		if(s.length() < 0.001f)
			s = r.cross(tgVector3(0,1,0));
		
		s.normalize();
		t = r.cross(s);
		
		Mt = tgMatrix3(r,s,t);
		M = tgMatrix3(Mt);
		M.transpose();	
		
		Rx = tgMatrix3(	1.0f,	0.0f,	0.0f,
											0.0f,	cos(alpha),	-sin(alpha),
											0.0f,	sin(alpha),	cos(alpha));
						
		X=Mt*Rx*M;

		v = tgVector3(x,y,z);
		n = X*v;
		
		x = n.x;
		y = n.y;
		z = n.z;
	}
}

float Angle(tgVector3 a, tgVector3 b)
{
	float d = a.dot(b)/(a.length() * b.length());
	return acos(d);
}



