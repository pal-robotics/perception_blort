
#include <blort/Tracker/Predictor.h>
#include <blort/Tracker/Noise.h>
#include <ros/console.h>

using namespace Tracking;

Predictor::Predictor(){
	m_dTime = 0.0;
	c_pred = 0.0;
	m_noConvergence = 0.0f;
}

// *** private ***
float Predictor::noise(float sigma, unsigned int type){
    float random = 0.0f;
        
    // Gaussian noise
    if(type == GAUSS){
		for(unsigned i=0; i<10; i++){
			random += float(rand())/RAND_MAX;
		}
		random = 2.0f * (random / 10.0f - 0.5f);
    }
    
    // Uniform distributed noise
    if(type == NORMAL){
    	random = 2.0f * (float(rand())/RAND_MAX - 0.5f);
    }
    
    // Adjusting to range
    random = random * sigma;
    
    return random;
}

Particle Predictor::genNoise(float sigma, Particle pConstraint, unsigned int type){
	if(sigma == 0.0f)
                ROS_DEBUG("[Predictor::genNoise] Warning standard deviation sigma is 0.0");
	
	Particle epsilon;
	Noise N;
	
	//epsilon.r.x = noise(sigma, type) * pConstraint.r.x;
	//epsilon.r.y = noise(sigma, type) * pConstraint.r.y;
	//epsilon.r.z = noise(sigma, type) * pConstraint.r.z;
	//epsilon.t.x = noise(sigma, type) * pConstraint.t.x;
	//epsilon.t.y = noise(sigma, type) * pConstraint.t.y;
	//epsilon.t.z = noise(sigma, type) * pConstraint.t.z;
	//
	//epsilon.z   = noise(sigma, type) * pConstraint.z;
	
	epsilon.r.x = N.randn_notrig(0.0f, sigma * pConstraint.r.x);
	epsilon.r.y = N.randn_notrig(0.0f, sigma * pConstraint.r.y);
	epsilon.r.z = N.randn_notrig(0.0f, sigma * pConstraint.r.z);
	epsilon.t.x = N.randn_notrig(0.0f, sigma * pConstraint.t.x);
	epsilon.t.y = N.randn_notrig(0.0f, sigma * pConstraint.t.y);
	epsilon.t.z = N.randn_notrig(0.0f, sigma * pConstraint.t.z);
	
	epsilon.z = N.randn_notrig(0.0f, sigma * pConstraint.z);
		
	return epsilon;
}

void Predictor::sampleFromGaussian(Distribution& d, int num_particles, Particle mean, Particle variance, float sigma){
	Particle p;
	Particle epsilon;
	
	d.push_back(mean);
	for(int i=1; i<num_particles; i++){
		
		// particle to sample from
		p = mean;
		
		// move function (gaussian noise)
		epsilon = genNoise(sigma, variance);
		
		// apply movement
		p.Translate(epsilon.t);
		p.RotateAxis(epsilon.r);
		p.Translate( m_cam_view.x * epsilon.z, m_cam_view.y * epsilon.z, m_cam_view.z * epsilon.z);

		// add particle to distribution
		d.push_back(p);
	}	
}

void Predictor::movePredicted(const TomGine::tgPose& poseIn, TomGine::tgPose& poseOut, float &c){
	poseOut = poseIn;
	c = 0.0;
}

// *** public ***

void Predictor::resample(Distribution& d, int num_particles, Particle variance, bool useMotion){
	
	if(d.size() <= 0)
		return;
	
	unsigned n, id;
//	unsigned nid=0;
	float sigma = 0.01f;
	float c=0.0f;
//	float cn=0.0f;
//	float c_pred_av = 0.0f;
	
	
	if(num_particles<=0){
                ROS_DEBUG("[Distribution::resample] Warning number of particles to low (0)");
		num_particles = d.size();
	}
	
	ParticleList particlelist_tmp;
	d.copy(particlelist_tmp);
	d.clear();
	
	//d.push_back(particlelist_tmp[0]);
	
	// Particles with motion
	for(	id=0; 
			id<particlelist_tmp.size() && 
			d.size()<(int)(num_particles*(1.0f - m_noConvergence)); 
			id++)
	{
		
		// resampling according to weight
		n = (unsigned)round(particlelist_tmp[id].w * num_particles);
		c = particlelist_tmp[id].c;
		
		// TODO evaluate if this makes sense (higher accuracy/convergence)
		// Remark: Defenitely removes jittering, maybe makes it less robust against fast movements
// 		if(d.getMaxC()>0.0f){
// 			cn = c / d.getMaxC();
// 			c = c*(1.0f-c) + cn*c;
//  			//c = 0.8*cn + c*(1.0-cn);
// 		}
		
		// Tukey estimator
		// TODO evaluate optimal power of estimator (accuracy / robustness)
// 		sigma = (1.0-pow(1.0-pow(1.0-c,2),3));
		sigma = 1.0f - c;
		
		// Prediction motion model
// 		if(useMotion){
// 			c_pred = 0.0f;
// 			movePredicted(particlelist_tmp[id], particlelist_tmp[id], c_pred);
// 		}
// 		if( c_pred>=0.0f && c_pred<=1.0f ){
// 			sigma = sigma * (1.0f - c_pred);
// 			c_pred_av += c_pred;
// 		}
		
		// Keep one particle of best matching particles
// 		if(id<unsigned(num_particles*(1.0f-m_noConvergence))>>2){
// 			d.push_back(particlelist_tmp[id]);
// 			n--;
// 		}
		
		// Noise motion model
		// ensure range of sigma
		if(sigma==0.0) sigma = 0.001f;
		if(sigma>1.0) sigma = 1.0f;
		sampleFromGaussian(d, n, particlelist_tmp[id], variance, sigma);
	}
	
	//// Particles voting for no motion
	//for(id=0; id<((int)num_particles*pNoMotion); id++){
	//	d.copyParticle(p, id);
	//	p.tp = vec3(0.0,0.0,0.0);
	//	p.rp = vec3(0.0,0.0,0.0);
	//	p.zp = 0.0;
	//	d.push_back(p);
	//}
	
	//// Particle voting for no convergence
	if(m_noConvergence > 0.0f){
		n = num_particles - d.size();
// 		Particle p = particlelist_tmp[0] * (1.0f - c_pred_av/id);
		Particle p = particlelist_tmp[0];
		sampleFromGaussian(d, n, p, variance, 1.0f);
	}
}

void Predictor::sample(Distribution& d, int num_particles, Particle mean, Particle variance){
	d.clear(); 
	sampleFromGaussian(d, num_particles, mean, variance);
}

void Predictor::updateTime(double dTime){
	m_dTime = dTime;
}
