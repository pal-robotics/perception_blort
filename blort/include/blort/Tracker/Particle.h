 /**
 * @file Particle.h
 * @author Thomas Mörwald
 * @date January 2010
 * @version 0.1
 * @brief Defining a point/particle in the likelihood distribution
 * @namespace Tracker
 */
 
#ifndef __PARTICLE_H__
#define __PARTICLE_H__

#include <blort/TomGine/tgPose.h>
#include <blort/TomGine/tgMathlib.h>
#include <blort/Tracker/headers.h>		// stdio.h

namespace Tracking{

/**	@brief class Particle */
class Particle : public TomGine::tgPose
{
public:
	vec3 r;				///< rotations
	float z;			///< scaling (zoom)
	
	std::vector<float> userData;
	
	float w;			///< weighted likelihood (sum of w of distribution = 1)
	float c;			///< confidence level (matching pixels divided by overall pixels)

	Particle(float val=0.0);
	Particle(vec3 t, vec3 r, float z, float w, float c, TomGine::tgQuaternion q);
	Particle(const Particle& p2);
	Particle(const TomGine::tgPose& p2);
	
	Particle& operator=(const Particle& p2);
	Particle& operator=(const TomGine::tgPose& p);
	Particle operator*(const float& f) const;
	Particle operator+(const Particle& p) const;
	Particle operator-(const Particle& p) const;
	
	/**	@brief Comparing weighted likelihood of two particles	*/
	inline bool operator<(const Particle& p2) const { return w < p2.w; }
	inline bool operator>(const Particle& p2) const { return w > p2.w; }

};

} // namespace Tracking

#endif
