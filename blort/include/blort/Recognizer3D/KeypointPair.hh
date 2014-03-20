/**
 * Johann Prankl, 2010-01-27 
 * prankl@acin.tuwien.ac.at
 */

#ifndef P_KEYPOINT_PAIR_HH
#define P_KEYPOINT_PAIR_HH

#include <blort/Recognizer3D/Keypoint.hh>

namespace P
{


class KeypointPair
{
public:
  Keypoint *k1;
  Keypoint *k2;

  KeypointPair() : k1(0), k2(0) {};
  KeypointPair(Keypoint *_k1, Keypoint *_k2) : k1(_k1), k2(_k2) {};
  ~KeypointPair(){};
};

}


#endif

