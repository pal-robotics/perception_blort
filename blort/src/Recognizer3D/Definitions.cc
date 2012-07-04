/**
 * $Id: Definitions.cc 34111 2012-07-03 14:29:54Z student5 $
 */

#include <blort/Recognizer3D/Definitions.hh>

using namespace P;

const int		Def::DO_MAX_RANSAC_TRIALS = 500; //= 1000;
const int		Def::DO_MATCHER = 0;     //0 = use match second nearest match, 1=threshold, 2=gpu
const float		Def::DO_MIN_DESC_DIST = 8.;          // descriptor distance for object detection 
const double	Def::DO_RANSAC_ETA0 = 0.01;         //failure probability
const double	Def::DO_RANSAC_INL_DIST = 2; //1.;
const float		Def::DO_CLUSTER_SIGMA_THR=0.2;         // cluster threshold for codebook
const double	Def::DO_TIME_MEAN = 5.;
const double	Def::DISTANCE_RATIO = 1.;   // least squares is only accepted if pose does not change very much

