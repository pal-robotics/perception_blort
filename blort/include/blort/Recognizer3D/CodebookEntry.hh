/**
 * Johann Prankl, 2010-01-27 
 * prankl@acin.tuwien.ac.at
 */

#ifndef P_CODEBOOK_ENTRY_HH
#define P_CODEBOOK_ENTRY_HH

#include <limits.h>
#include <blort/Recognizer3D/PNamespace.hh>
#include <blort/Recognizer3D/Array.hh>
#include <blort/Recognizer3D/KeypointDescriptor.hh>

namespace P
{

class KeypointDescriptor;

class CodebookEntry
{
public:
  float sqr_sigma;
  KeypointDescriptor *model;
  P::Array<KeypointDescriptor* > occurrences;

  bool good;          //-1 false match, 0 false and correct match, 1 only correct match
  bool bad;
  int cntGood;       // count good matches
  int cntTime;       // ''timestamp''
  double reliability;

  CodebookEntry();
  CodebookEntry(KeypointDescriptor *k);
  ~CodebookEntry();

  inline unsigned Size(){return occurrences.Size();}
  inline void Clear();
  inline KeypointDescriptor *Insert(KeypointDescriptor* oc);
  inline bool Insert(KeypointDescriptor* occ, float sqr_sigma); 
  inline void ComputeModel();
  inline float CombinedSqrSigma(CodebookEntry *c);
  inline float CombinedSqrSigma(KeypointDescriptor *occ);
  inline bool Combine(CodebookEntry *c);
  inline float DistanceSqr(CodebookEntry *c);

};

inline void DeleteCodebook(Array<CodebookEntry*> &codebook);

}

#include <blort/Recognizer3D/CodebookEntry.ic>

#endif

