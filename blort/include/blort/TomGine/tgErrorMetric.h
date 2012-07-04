
#ifndef ERROR_METRIC_H
#define ERROR_METRIC_H

#include <vector>

#include <blort/TomGine/tgModel.h>
#include <blort/TomGine/tgPose.h>
#include <blort/TomGine/tgMathlib.h>

namespace TomGine{

class tgErrorMetric
{
private:
	
	
	vec3 GetRandPointInTriangle(const vec3& t1, const vec3& t2, const vec3& t3, unsigned trials=100) const;
	
public:
	std::vector<vec3> pointlist;
	
	tgErrorMetric(TomGine::tgModel model, unsigned num_points=10000);
	
	vec3 Compare(const TomGine::tgPose &p1, const TomGine::tgPose &p2);
	
	void GetPoints(std::vector<vec3> &pl){ pl = pointlist; }
	
	inline unsigned GetNumPoints(){ return pointlist.size(); }
	
};

} // namespace TomGine

#endif
