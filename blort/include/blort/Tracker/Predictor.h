/**
* @file Predictor.h
* @author Thomas Mörwald
* @date January 2009
* @version 0.1
* @brief Prediction for motion of object to track
* @namespace Tracker
*/

#ifndef __PREDICTOR_H__
#define __PREDICTOR_H__

#include <blort/Tracker/headers.h>
#include <blort/Tracker/Distribution.h>
#include <blort/TomGine/tgVector3.h>
#include <blort/Tracker/Timer.h>

namespace Tracking{

#define GAUSS  0
#define NORMAL 1

/** @brief class Predictor */
class Predictor
{
protected:
	double m_dTime;
	float m_powTime;
	float m_powTimeSteps;
	float c_pred;
	
	float m_noConvergence;
	
	TomGine::tgVector3 m_cam_view;
	
	float noise(float sigma, unsigned int type=GAUSS);
	Particle genNoise(float sigma, Particle pConstraint, unsigned int type=GAUSS);

public:
	Predictor();
	
	/** @brief Set vector pointing from camera to object mean, to enable zooming*/
	void setCamViewVector(TomGine::tgVector3 v){ m_cam_view = v; m_cam_view.normalize(); }
	
	/** @brief Set the number of particles (in percent) which are voting for no convergence (for capturing fast movement)*/
	void setNoConvergence(float v){ if(v>=0.0f && v<=1.0f) m_noConvergence = v; }
	
	/** @brief Adds samples/particles to a distribution d by adding gaussian noise
	*		@param d particle distribution
	*		@param num_particles number of particles of resampled likelihood distribution
	*		@param mean mean particle of distribution
	*		@param variance variance of sampling in each degree of freedom (represented as particle)
	*		@param sigma standard variation of sampling (dependent on confidence value -> sigma(c)) */
	virtual void sampleFromGaussian(Distribution& d, int num_particles, Particle mean, Particle variance, float sigma=1.0);
	
	/** @brief Interface for Prediction/Motion systems
	*		@param poseIn actual pose
	*		@param poseOut predicted/moved pose
	*		@param c confidence about prediction/movement ( 1.0 = high confidence; 0.0 = no confidence ) */
	virtual void movePredicted(const TomGine::tgPose& poseIn, TomGine::tgPose& poseOut, float &c);
	
	/**	@brief Resample particles according to current likelihood distribution (move particles)
	*		@param d pointer to distribution
	*		@param num_particles number of particles of resampled likelihood distribution
	*		@param variance variance of sampling in each degree of freedom (represented as particle) */
	virtual void resample(Distribution& d, int num_particles, Particle variance, bool useMotion=true);
	
	/** @brief Sample new distribution 
	*		@param d pointer to distribution
	*		@param num_particles number of particles to represent likelihood distribution
	*		@param mean mean particle of distribution
	*		@param variance variance of sampling in each degree of freedom (represented as particle) */
	virtual void sample(Distribution& d, int num_particles, Particle mean, Particle variance);
	
	/** @brief Updates time of prediction system for higher order motion model
	*		@param dTime Time since last frame */
	virtual void updateTime(double dTime);
	
};

} /* namespace Tracking */
 
 #endif
