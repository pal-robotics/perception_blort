/**
 * $Id: Vector2.cc 34111 2012-07-03 14:29:54Z student5 $
 */

#include <blort/Recognizer3D/Vector2.hh>

namespace P 
{

/**
 * Returns intersection of lines defined by point p and direction d (needs not
 * be a unit vector).
 */
Vector2 LineIntersection(const Vector2 &p1, const Vector2 &d1,
    const Vector2 &p2, const Vector2 &d2) throw(Except)
{
  double d = Cross(d2, d1);
  if(d == 0.)
    throw Except(__HERE__, "lines do not intersect");
  double l = Cross(d2, p2 - p1)/d;
  return Vector2(p1.x + l*d1.x, p1.y + l*d1.y);
}

/**
 * Returns intersection of lines defined by point p and direction d (needs not
 * be a unit vector).
 * l1 and l2 contain the lengths along the lines where the intersection lies,
 * measured in lengths of d1 and d2.
 * So if You want l1 and l2 in pixels, d1 and d2 must be unit vectors.
 */
Vector2 LineIntersection(const Vector2 &p1, const Vector2 &d1,
    const Vector2 &p2, const Vector2 &d2, double *l1, double *l2) throw(Except)
{
  double d = Cross(d2, d1);
  if(d == 0.)
    throw Except(__HERE__, "lines do not intersect");
  Vector2 p12 = p2 - p1;
  *l1 = Cross(d2, p12)/d;
  *l2 = Cross(d1, p12)/d;
  return Vector2(p1.x + *l1*d1.x, p1.y + *l1*d1.y);
}

/**
 * Checks wheter lines a and b defined by their end points intersect.
 */
bool LinesIntersecting(const Vector2 &a1, const Vector2 &a2,
    const Vector2 &b1, const Vector2 &b2)
{
  double l1 = 0., l2 = 0.;
  LineIntersection(a1, a2 - a1, b1, b2 - b1, &l1, &l2);
  return (l1 >= 0. && l1 <= 1.) && (l2 >= 0. && l2 <= 1.);
}

/**
 * Checks wheter lines a and b defined by their end points intersect.
 */
bool LinesIntersecting(const Vector2 &a1, const Vector2 &a2,
    const Vector2 &b1, const Vector2 &b2, Vector2 &isct)
{
  double l1 = 0., l2 = 0.;
  isct=LineIntersection(a1, a2 - a1, b1, b2 - b1, &l1, &l2);
  return (l1 >= 0. && l1 <= 1.) && (l2 >= 0. && l2 <= 1.);
}

}

