/**
 */

#ifndef P_DEFINITIONS_HH
#define P_DEFINITIONS_HH

#pragma once

#include <blort/Recognizer3D/PNamespace.hh>

//for debugging
//#define DEBUG

#define NUM_THREADS 2



namespace P
{

class Def
{
public:
  static const int DO_MAX_RANSAC_TRIALS;
  static const int DO_MATCHER;     //0 = use match second nearest match, 1=threshold, 2=gpu
  static const float DO_MIN_DESC_DIST;          // descriptor distance for object detection 
  static const double DO_RANSAC_ETA0;         //failure probability
  static const double DO_RANSAC_INL_DIST; //1.;
  static const float DO_CLUSTER_SIGMA_THR;         // cluster threshold for codebook
  static const double DO_TIME_MEAN;
  static const double DISTANCE_RATIO;   // least squares is only accepted if pose does not change very much
};

}
#endif
