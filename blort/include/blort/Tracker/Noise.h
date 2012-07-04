

#ifndef _TRACKING_NOISE_
#define _TRACKING_NOISE_

 
namespace Tracking{

class Noise
{
public:

	//	"Polar" version without trigonometric calls
	float randn_notrig(float mu=0.0f, float sigma=1.0f);

	//	Standard version with trigonometric calls
	float randn_trig(float mu=0.0f, float sigma=1.0f);

	//	"Polar" version without trigonometric calls
	double randn_notrig(double mu=0.0, double sigma=1.0);

	//	Standard version with trigonometric calls
	double randn_trig(double mu=0.0, double sigma=1.0);

};

} 
 
#endif
