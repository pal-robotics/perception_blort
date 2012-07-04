/**
* @file tgCollission.h
* @author Thomas Mörwald
* @date June 2010
* @version 0.1
* @brief Intersection tests between objects (Point, Line, Ray, Sphere, Boxes, ...)
* @namespace TomGine
*/

#ifndef TG_COLLISSION_H
#define TG_COLLISSION_H

#include <vector>
#include <blort/TomGine/tgMathlib.h>
#include <blort/TomGine/tgModel.h>
#include <blort/TomGine/tgRenderModel.h>

namespace TomGine{

class tgCollission{
private:


public:
	static vec3 GetNormal(const vec3& v1, const vec3& v2, const vec3& v3);
	
	static bool IntersectRayTriangle(vec3& p, vec3& n, double& z, const tgRay& ray, const vec3& t1, const vec3& t2, const vec3& t3);
	static bool IntersectRayModel(std::vector<vec3> &pl, std::vector<vec3> &nl, std::vector<double> &zl, const tgRay& ray, const tgModel& model);
	
	static bool IntersectModels(const tgModel &m1, const tgModel &m2, const tgPose &p1, const tgPose& p2);
	static bool IntersectModels(const tgRenderModel &m1, const tgRenderModel &m2);
	
 	static bool PointOnSameSide(const vec3& p1, const vec3& p2, const vec3& a, const vec3& b);
	static bool PointInTriangle(const vec3& p, const vec3& t1, const vec3& t2, const vec3& t3);
 	
};

}
 
 #endif /* TG_COLLISSION_H */
