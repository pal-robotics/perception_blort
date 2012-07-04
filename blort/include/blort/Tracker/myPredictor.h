/**
* @file Predictor.h
* @author Thomas Mörwald
* @date January 2009
* @version 0.1
* @brief Prediction for motion of object to track
* @namespace Tracker
*/

#ifndef _MY_PREDICTOR_H_
#define _MY_PREDICTOR_H_

#include <blort/Tracker/headers.h>
#include <blort/Tracker/Predictor.h>

namespace Tracking{

/** @brief class Predictor */
class myPredictor : public Predictor
{
protected:
	virtual void sampleFromGaussian(Distribution& d, int num_particles, Particle p_initial, Particle p_constraints, float sigma=1.0);
	
};

}
 
 #endif
