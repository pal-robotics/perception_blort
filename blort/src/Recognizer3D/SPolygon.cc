/**
 * $Id: SPolygon.cc 34111 2012-07-03 14:29:54Z student5 $
 * Johann Prankl, 2010-01-27 
 * prankl@acin.tuwien.ac.at
 */


#include <blort/Recognizer3D/SPolygon.hh>
#include <blort/Recognizer3D/Geometry.hh>
#include <float.h>
#include <stdio.h>
#include <fstream>
#include <iostream>

namespace P 
{

SPolygon::SPolygon()
{
  p = UNDEF_ID;
  center = Vector2(DBL_MAX,DBL_MAX);
}

#ifdef USE_VS_CLOSURES
SPolygon::SPolygon(Closure *c)
{
  p = UNDEF_ID;
  v = c->v;
  center = Vector2(DBL_MAX,DBL_MAX);

}
#endif

SPolygon::SPolygon(Array<Vector2> &p)
{
  v = p;
  center = Vector2(DBL_MAX,DBL_MAX);
  //SetConvexHull(p);
}

SPolygon& SPolygon::operator=(const SPolygon &p)
{
  this->v = p.v;
  this->p = p.p;
  this->center = p.center;
  return *this;
}

/**
 * SetConvexHull
 * Calculates the convex hull of p
 */
void SPolygon::SetConvexHull(Array<Vector2> &p)
{
  ChainHull2D( p, v);
}

void SPolygon::Expand(double d)
{
  double angle;
  double h2;
  int j;
  int jm1;
  int jp1;
  Vector2 p4;
  int n=v.Size();

  Array<Vector2> eh;

  //  Consider each angle, formed by the nodes P(I-1), P(I), P(I+1).
  for ( j = 0; j < n; j++ )
  {
    jm1 = IWrap ( j-1, 0, n-1 );
    jp1 = IWrap ( j+1, 0, n-1 );
//
//        P1
//        /
//       /   P4
//      /  .
//     / .
//    P2--------->P3
    AngleHalf (v[jm1],v[j],v[jp1],p4);

//  Compute the value of the half angle.
    angle = AngleRAD (v[jm1],v[j],p4);
//  The stepsize along the ray must be adjusted so that the sides
//  move out by H.
    h2 = d / sin(angle);
    eh.PushBack(Vector2(v[j].x-h2*(p4.x-v[j].x), v[j].y-h2*(p4.y-v[j].y)));
  }
  v=eh;
}

/**
 * Checks whether point p is inside polygon.
 * Uses the Jordan curve theorem.
 * Note that points on the boundary are undefined.
 * Code thanks to
 *  W Randolph Franklin (WRF)
 *  http://www.ecse.rpi.edu/Homepages/wrf/Research/Short_Notes/pnpoly.html
 *  int pnpoly(int npol, float *xp, float *yp, float x, float y)
 *  {
 *    int i, j, c = 0;
 *    for (i = 0, j = npol-1; i < npol; j = i++) {
 *      if ((((yp[i]<=y) && (y<yp[j])) ||
 *           ((yp[j]<=y) && (y<yp[i]))) &&
 *          (x < (xp[j] - xp[i]) * (y - yp[i]) / (yp[j] - yp[i]) + xp[i]))
 *
 *        c = !c;
 *    }
 *    return c;
 *  }
 */
bool SPolygon::Inside(double x, double y)
{
  int i, j, npol = v.Size();
  bool c = false;
  for(i = 0, j = npol-1; i < npol; j = i++)
  {
    if((((v[i].y <= y) && (y < v[j].y)) ||
        ((v[j].y <= y) && (y < v[i].y))) &&
       (x < (v[j].x - v[i].x) * (y - v[i].y) / (v[j].y - v[i].y) + v[i].x))
      c = !c;
  }
  return c;
}

bool SPolygon::OnContour(double x, double y, double eps)
{
  for (unsigned i=0; i<v.Size(); i++)
  {
    if (Distance(v[i],P::Vector2(x,y)) < eps)
      return true;
  }
  return false;
}

bool SPolygon::Inside(Array<Vector2> &vs, Vector2 &p)
{
  int i, j, npol = vs.Size();
  bool c = false;
  for(i = 0, j = npol-1; i < npol; j = i++)
  {
    if((((vs[i].y <= p.y) && (p.y < vs[j].y)) ||
        ((vs[j].y <= p.y) && (p.y < vs[i].y))) &&
       (p.x < (vs[j].x - vs[i].x) * (p.y - vs[i].y) / (vs[j].y - vs[i].y) + vs[i].x))
      c = !c;
  }
  return c;
}

/**
 * search for nearest neighbour of a point in 2d
 */
double SPolygon::NearestNeighbour(Vector2 &p, Array<Vector2> &vs, unsigned &id)
{
  double dist, min_dist=DBL_MAX;
  for (unsigned i=0; i<vs.Size(); i++){
    dist = Distance(p, vs[i]);
    if (dist<min_dist){
      min_dist = dist;
      id = i;
    }
  }
  return min_dist;
}

/**
 * search for nearest line to point (normal length between two end points)
 */
double SPolygon::NearestLine(Vector2 &p, Array<Vector2> &vs, unsigned &id)
{
  id=UINT_MAX;
  double dist, min_dist=DBL_MAX;
  Vector2 d, l2p;

  for (unsigned i=0; i<vs.Size(); i++){
    d = vs[vs.CircularNext(i)]-vs[i];
    d.Normalise();
    l2p = p-vs[i];

    dist = fabs(Cross(l2p, d));
    if (Dot(l2p,d)>0. && dist<min_dist){
      min_dist=dist;
      id=i;
    }
  }

  return min_dist;
}

/**
 * @brief Match two polygons using Hausdorff distance
 */
double SPolygon::Match(Array<Vector2> &vs1, Array<Vector2> &vs2)
{
  unsigned z;
  double dist, h_dist=0., min_dist=DBL_MAX;

  //search for each vs1 vertex in vs2 
  for (unsigned i=0; i<vs1.Size(); i++){
    //search for nearest neighbour to vertex
    min_dist = NearestNeighbour(vs1[i], vs2, z);
    //search for nearest line (normal distance between two end points)
    dist = NearestLine(vs1[i], vs2, z);
    if (dist<min_dist) min_dist=dist;
    //check for minimum
    if (min_dist>h_dist) h_dist=min_dist;
  }

  //search for each vs2 vertex in vs1 
  for (unsigned i=0; i<vs2.Size(); i++){
    //search for nearest neighbour to vertex
    min_dist = NearestNeighbour(vs2[i], vs1, z);
    //search for nearest line (normal distance between two end points)
    dist = NearestLine(vs2[i], vs1, z);
    if (dist<min_dist) min_dist=dist;
    //check for minimum
    if (min_dist>h_dist) h_dist=min_dist;
  }

  return h_dist;
}

/**
 * @brief Calculate center of gravity
 */
void SPolygon::CenterOfGravity(Array<Vector2> &vs, Vector2 &vc)
{
  vc=P::Vector2(0.,0.);
  if (vs.Size()>0)
  {
    for (unsigned i=0; i<vs.Size(); i++)
      vc+=vs[i];
    vc/=vs.Size();
  }
}

/**
 * @brief Calculate area of polygon
 */
double SPolygon::Area(Array<Vector2> &vs)
{
  double area;
  int im1;
  int ip1;

  area = 0.0;

  for ( unsigned i = 1; i <= vs.Size(); i++ )
  {
    im1 = IWrap ( i-1, 1, vs.Size() );
    ip1 = IWrap ( i+1, 1, vs.Size() );

    area = area + vs[i-1].x * ( vs[ip1-1].y - vs[im1-1].y );
  }

  area = 0.5 * fabs ( area );

  return area;

}


void SPolygon::Save(ofstream &os, const SPolygon &p)
{
  os<<p.v.Size();

  for (unsigned i=0; i<p.v.Size(); i++)
    os<<' '<<p.v[i].x<<' '<<p.v[i].y;

  os<<' '<<p.center.x<<' '<<p.center.y;
  os<<' '<<p.p<<'\n';
}

void SPolygon::Load(ifstream &is, SPolygon &p)
{
  unsigned size;
  is >> size;
  p.v.Resize(size);
  for (unsigned i=0; i<size; i++)
    is >>p.v[i].x>>p.v[i].y;
  is >>p.center.x>>p.center.y;
  is >>p.p;
}





}

