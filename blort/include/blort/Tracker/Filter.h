

#ifndef _TRACKING_SMOOTH_FILTER_
#define _TRACKING_SMOOTH_FILTER_

 
namespace Tracking{

class SmoothFilter
{
public:
    SmoothFilter(){ a = 0.0f; b = 1.f - a; z = 0; };
    SmoothFilter(float a){ this->a = a; b = 1.f - a; z = 0; };
    inline void reset(float a) { this->a = a; b = 1.f - a; z = 0; };
    inline float Process(float in) { z = (in * b) + (z * a); return z; }
        inline void Set(const float &z) { this->z = z; }
        inline void Set(const float &z, float delay) { this->z = z; a = delay; b = 1.f - delay; }
	inline void SetDelay(float d) { a = d; b = 1.f- d; }
private:
    float a, b, z;
};


} 
 
#endif
