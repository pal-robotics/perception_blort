
#include <blort/TomGine/tgCollission.h>
#include <stdexcept>

using namespace TomGine;

vec3 tgCollission::GetNormal(const vec3& v1, const vec3& v2, const vec3& v3)
{
	vec3 n, e1, e2;
	
	e1 = v2 - v1;
	e2 = v3 - v1;
	
	n.cross(e1, e2);
	n.normalize();
	
	return n;
}

bool tgCollission::IntersectRayTriangle(vec3& p, vec3& n, double& z, const tgRay& ray, const vec3& t1, const vec3& t2, const vec3& t3)
{
	// p = ray.start + ray.dir * z (1)
	// p*n + d = 0   ; point p must be on plane (with normal vector n and distance d) (2)
	
	n = GetNormal(t1, t2, t3); // plane normal
	
	double d = - t1 * n;	// distance of plane form origin 
	
	// from (1) and (2)
	z = -( ray.start * n + d ) / (ray.dir * n);

	p = ray.start + ray.dir * (float)z;
	
	// test if point lies in triangle or not
	return PointInTriangle(p, t1, t2, t3);
}

bool tgCollission::IntersectRayModel(std::vector<vec3> &pl, std::vector<vec3> &nl, std::vector<double> &zl, const tgRay& ray, const tgModel& model)
{
	bool hit = false;
	pl.clear();
	zl.clear();
	
	for(unsigned int i=0; i<model.m_faces.size(); i++)
	{
		vec3 p;
		vec3 n;
		double z;
		
		if(model.m_faces[i].v.size()<3)
			throw std::runtime_error("[tgCollission::IntersectRayModel] Error number of vertices to low (<3) for a face");
		
		vec3 t1 = model.m_vertices[model.m_faces[i].v[0]].pos;
		vec3 t2 = model.m_vertices[model.m_faces[i].v[1]].pos;
		vec3 t3 = model.m_vertices[model.m_faces[i].v[2]].pos;
		
		if(IntersectRayTriangle(p, n, z, ray, t1, t2, t3))
		{
			pl.push_back(p);
			nl.push_back(n);
			zl.push_back(z);
			hit = true;
		}
		
		if(model.m_faces[i].v.size()==4)
		{
			t1 = model.m_vertices[model.m_faces[i].v[2]].pos;
			t2 = model.m_vertices[model.m_faces[i].v[3]].pos;
			t3 = model.m_vertices[model.m_faces[i].v[0]].pos;
			
			if(IntersectRayTriangle(p, n, z, ray, t1, t2, t3))
			{
				pl.push_back(p);
				nl.push_back(n);
				zl.push_back(z);
				hit = true;
			}
		}
		
		if(model.m_faces[i].v.size()>4)
			throw std::runtime_error("[tgCollission::IntersectRayModel] Error number of vertices to low (<3) for a face");
		
	}
	return hit;
}

bool tgCollission::IntersectModels(const tgModel &m1, const tgModel &m2, const tgPose &p1, const tgPose& p2)
{
	vec3 t1, t2;
	mat3 R1, R2;
	
	p1.GetPose(R1, t1);
	p2.GetPose(R2, t2);
  
	vec3 center1 = t1 + R1 * m1.m_bs.center;
	vec3 center2 = t2 + R2 * m2.m_bs.center;
	
	float d = (center1 - center2).length();
	
// 	printf("tgCollission::IntersectModels: %f %f\n", d, (m1.m_bs.radius+m2.m_bs.radius));
	
	if(d >= (m1.m_bs.radius+m2.m_bs.radius))
		return false;
	
	// further tests
	return true;
}

bool tgCollission::IntersectModels(const tgRenderModel &m1, const tgRenderModel &m2)
{
	vec3 t1, t2;
	mat3 R1, R2;
	
	m1.m_pose.GetPose(R1, t1);
	m2.m_pose.GetPose(R2, t2);
  
	vec3 center1 = t1 + R1 * m1.m_bs.center;
	vec3 center2 = t2 + R2 * m2.m_bs.center;
	
	float d = (center1 - center2).length();
	
// 	printf("tgCollission::IntersectModels: %f %f\n", d, (m1.m_bs.radius+m2.m_bs.radius));
	
	if(d >= (m1.m_bs.radius+m2.m_bs.radius))
		return false;
	
	// further tests
	return true;
}


bool tgCollission::PointOnSameSide(const vec3& p1, const vec3& p2, const vec3& a, const vec3& b)
{
	vec3 cp1, cp2;
	cp1.cross(b-a, p1-a);
	cp2.cross(b-a, p2-a);
		
        return ((cp1*cp2) >= 0.0);
}

bool tgCollission::PointInTriangle(const vec3& p, const vec3& t1, const vec3& t2, const vec3& t3)
{

return(	PointOnSameSide( p, t1, t2, t3 ) &&
				PointOnSameSide( p, t2, t3, t1 ) &&
				PointOnSameSide( p, t3, t1, t2 ));
}

