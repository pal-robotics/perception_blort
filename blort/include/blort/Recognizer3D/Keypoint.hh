/**
 * Johann Prankl, 2010-01-27 
 * prankl@acin.tuwien.ac.at
 */

#ifndef P_KEYPOINT_HH
#define P_KEYPOINT_HH

#include <blort/Recognizer3D/PNamespace.hh>
#include <blort/Recognizer3D/Vector2.hh>
#include "float.h"

namespace P
{

class Keypoint
{
public:
  Vector2 p;            //location of the keypoint with respect to center of the object
  float scale;
  float angle;
  float mi11,mi12,mi21,mi22;

  Keypoint *bw, *fw;     //backward/forward link for tracking

  int nb;
  static int nbcnt;
  unsigned id;
  static unsigned idcnt;

  float error;

  Keypoint();
  Keypoint(Keypoint *k);
  Keypoint(double x, double y);
  Keypoint(double x, double y, float s, float a);
  Keypoint(double x, double y, Keypoint *bw);
  Keypoint(double x,double y,float s,float a,float _m11,float _m12,float _m21,float _m22);
  ~Keypoint();

  inline double X() {return p.x;}
  inline double Y() {return p.y;}
  inline float Scale() {return scale;}
  inline float Angle() {return angle;}
  inline float Mi11() {return mi11;}
  inline float Mi12() {return mi12;}
  inline float Mi21() {return mi21;}
  inline float Mi22() {return mi22;} 
};

/******************************** INLINE METHODES ***************************/







} //--END--

#endif

