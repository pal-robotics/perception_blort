/**
 * $Id: Keypoint.cc 34111 2012-07-03 14:29:54Z student5 $
 * Johann Prankl, 2010-01-27 
 * prankl@acin.tuwien.ac.at
 */


#include <blort/Recognizer3D/Keypoint.hh>

namespace P 
{

int Keypoint::nbcnt=0;
unsigned Keypoint::idcnt=0;


/**
 * Constructor/Destructor
 */
Keypoint::Keypoint()
 : mi11(0), mi12(1), mi21(1), mi22(0), bw(0), fw(0), nb(0), id(UINT_MAX)
{
  scale = 1.;
  angle = 0.;
  error = FLT_MAX;
}

Keypoint::Keypoint(double x, double y)
 : p(P::Vector2(x,y)), mi11(0), mi12(1), mi21(1), mi22(0), bw(0), fw(0), 
   nb(0), id(UINT_MAX)
{
  scale = 1.;
  angle = 0.;
  error = FLT_MAX;
}

Keypoint::Keypoint(double x, double y, Keypoint *b)
 : p(P::Vector2(x,y)),  mi11(0), mi12(1), mi21(1), mi22(0), bw(b), fw(0), 
   nb(0), id(UINT_MAX)
{
  scale = 1.;
  angle = 0.;
  error = FLT_MAX;
}

Keypoint::~Keypoint()
{
}

Keypoint::Keypoint(Keypoint *k)
{
  p=k->p;
  scale=k->scale;
  angle=k->angle;
  mi11=k->mi11;
  mi12=k->mi12;
  mi21=k->mi21;
  mi22=k->mi22;
  bw=k->bw;
  fw=k->fw;
  nb=k->nb;
  id=k->id;
  error=k->error;
}

Keypoint::Keypoint(double x, double y, float s, float a)
 : p(P::Vector2(x,y)), scale(s), angle(a), mi11(1), mi12(0), mi21(0), mi22(1),
   bw(0), fw(0), nb(0), id(UINT_MAX)
{
  error=FLT_MAX;
}

Keypoint::Keypoint(double x, double y, float s, float a,
                   float _m11,float _m12,float _m21,float _m22)
 : p(P::Vector2(x,y)), scale(s),angle(a),mi11(_m11),mi12(_m12),mi21(_m21),mi22(_m22),
   bw(0), fw(0), nb(0), id(UINT_MAX)
{
  error=FLT_MAX;
}

}

