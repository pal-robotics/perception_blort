/**
 * Johann Prankl, 2010-01-27 
 * prankl@acin.tuwien.ac.at
 */

#ifndef P_OBJECT_3D_HH
#define P_OBJECT_3D_HH

//#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include <map>
#include <blort/Recognizer3D/PNamespace.hh>
#include <blort/Recognizer3D/Array.hh>
#include <blort/Recognizer3D/PoseCv.hh>
#include <blort/Recognizer3D/CodebookEntry.hh>
#include <blort/Recognizer3D/SPolygon.hh>

namespace P
{


class Object3D
{
public:
  unsigned id;
  static unsigned idcnt;
 
  Array<CodebookEntry *> codebook;

  double conf;       //confidence value [0...1]
  double err;        //reprojection error [0...Def::DO_RANSAC_INL_DIST]

  PoseCv pose;  
  SPolygon contour;

  Object3D();
  ~Object3D();
};


void DeleteObjects3D(Array<Object3D*> &objects);


/*********************** INLINE METHODES **************************/



}

#endif

