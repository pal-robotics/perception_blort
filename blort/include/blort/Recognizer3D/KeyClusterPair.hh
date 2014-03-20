/**
 * Johann Prankl, 2010-01-27 
 * prankl@acin.tuwien.ac.at
 */

#ifndef P_KEY_CLUSTER_PAIR_HH
#define P_KEY_CLUSTER_PAIR_HH

#include <blort/Recognizer3D/KeypointDescriptor.hh>
#include <blort/Recognizer3D/CodebookEntry.hh>

namespace P
{


class KeyClusterPair
{
public:
  KeypointDescriptor *k;
  CodebookEntry *c;
  float dist;

  KeyClusterPair() : k(0), c(0) {};
  KeyClusterPair(KeypointDescriptor *_k, CodebookEntry *_c) : k(_k), c(_c) {};
  KeyClusterPair(KeypointDescriptor *_k, CodebookEntry *_c, float d) : k(_k), c(_c), dist(d) {};
  ~KeyClusterPair(){};
};

}


#endif

