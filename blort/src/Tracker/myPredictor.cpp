
#include <blort/Tracker/myPredictor.h>

using namespace Tracking;


void myPredictor::sampleFromGaussian(Distribution& d, int num_particles, Particle mean, Particle variance, float sigma){
	Particle p;
	Particle epsilon;
	
	for(int i=0; i<num_particles; i++){
		
		p = mean;
		
		epsilon = genNoise(sigma, variance);
		
		p.Translate( m_cam_view.x * p.z, m_cam_view.y * p.z, m_cam_view.z * p.z);
		
		d.push_back(p);
		
		
	}	
}
