/**
 * Johann Prankl, 2010-01-27 
 * prankl@acin.tuwien.ac.at
 */

#ifndef P_SPOLYGON_HH
#define P_SPOLYGON_HH

//#define USE_VS_CLOSURES

#include <float.h>
#include <map> 
#include <blort/Recognizer3D/Vector2.hh>
#include <blort/Recognizer3D/PNamespace.hh>
#include <blort/Recognizer3D/Array.hh>

#ifdef USE_VS_CLOSURES
#include <blort/Recognizer3D/Closure.hh>
#endif

namespace P
{

class SPolygon
{
public:
  Array<Vector2> v;
  Vector2 center;       //center of gravity
  unsigned p;           //pointer to a proto object 
  
  SPolygon();
	#ifdef USE_VS_CLOSURES
  SPolygon(Closure *c);
	#endif
  SPolygon(Array<Vector2> &p);
  ~SPolygon(){};
  void SetConvexHull(Array<Vector2> &p);
  void Expand(double d); 
  bool Inside(double x, double y);
  bool OnContour(double x, double y, double eps);
  inline bool Inside(Vector2 &p) {return Inside(p.x,p.y);}
  inline void Clear(){v.Clear();}
  inline unsigned Size(){return v.Size();}
  inline void PushBack(Vector2 &p){v.PushBack(p);}
  inline unsigned NumVertices(){return v.Size();}
  inline void Insert(Vector2 &p){v.PushBack(p);}
  inline void Move(Vector2 &dist);
  inline double Area(){return Area(v);}
  void CenterOfGravity();

  static bool Inside(Array<Vector2> &vs, Vector2 &p);
  static double NearestLine(Vector2 &p, Array<Vector2> &vs, unsigned &id);
  static double NearestNeighbour(Vector2 &p, Array<Vector2> &vs, unsigned &id);
  static double Match(Array<Vector2> &vs1, Array<Vector2> &vs2);
  static void CenterOfGravity(Array<Vector2> &vs, Vector2 &vc);
  static double Area ( Array<Vector2> &vs );
  static void Load(ifstream &is, SPolygon &p);
  static void Save(ofstream &os, const SPolygon &p);

  SPolygon &operator=(const SPolygon &p);
  Vector2& operator[](unsigned i){return v[i];}
};


/************************ INLINE METHODES *******************************/
inline void SPolygon::CenterOfGravity()
{
  if (v.Size()>0){
    center=P::Vector2(0.,0.);
    for (unsigned i=0; i<v.Size(); i++)
      center+=v[i];
    center/=v.Size();
  }
}

/**
 * @brief Move the whole polygon (and center) by dist
 */
inline void SPolygon::Move(Vector2 &dist)
{
  if (center.x!=DBL_MAX)
    center+=dist;
  for (unsigned i=0; i<v.Size(); i++)
    v[i]+=dist;
}


}

#endif

