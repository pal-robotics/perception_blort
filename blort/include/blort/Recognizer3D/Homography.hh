/**
 * Johann Prankl, 2010-03-30
 * prankl@acin.tuwien.ac.at
 */

#ifndef P_HOMOGRAPHY_HH
#define P_HOMOGRAPHY_HH

#include <iostream>
#include <float.h>
//#include <opencv/cv.h>
//#include <opencv/cxcore.h>
#include <opencv2/opencv.hpp>
#include <blort/Recognizer3D/Vector2.hh>
#include <blort/Recognizer3D/Array.hh>
#include <blort/Recognizer3D/PNamespace.hh>
#include <blort/Recognizer3D/Except.hh>
#include <blort/Recognizer3D/homest.h>
#include <blort/Recognizer3D/Math.hh>
#include <blort/Recognizer3D/SMatrix.hh>
#include <blort/Recognizer3D/PoseCv.hh>

namespace P 
{

class Homography 
{
private:
  Array<Vector2> pts1, pts2;
  Array<Vector2> lns1, lns2;

  unsigned lastPt;           //pointer to the last point
  unsigned lastLn;           //pointer to the last line

  double *T1, *invT;

  void NormalizePts(Array<Vector2> &in, unsigned numPts, Array<Vector2> &out, double T[9]);
  void NormalizeLns(double T[9], Array<Vector2> &in, unsigned numLns, Array<Vector2> &out);
  void SetDataMatrixPts(Array<Vector2> &pts1, Array<Vector2> &pts2, unsigned numPts, unsigned start, CvMat *A);
  void SetDataMatrixLns(Array<Vector2> &lns1, Array<Vector2> &lns2, unsigned numLns, unsigned start, CvMat *A);
  void NormalizeH(double H[9], double T1[9], double T2[9], double nH[9]);
  void DenormalizeH(double nH[9], double T1[9], double T2[9], double H[9]);
  void GetRandIdx(unsigned size, unsigned num, P::Array<unsigned> &idx);
  bool SolveHomography(CvMat *A, CvMat *W, CvMat *Ut, CvMat *Vt, double H[9]);
  void CountInlierPts(double H[9], double thr, int &inl);
  void CountInlierLns(double H[9], double thr, double thrAngle, int &inl);
  void SetDataMatrix(double *d, double H[9], double thr, double thrAngle, int &numInl);


  inline void InsertPointCoeff(Vector2 &pt1, Vector2 &pt2, double *d);
  inline void InsertLineCoeff(Vector2 &ln1, Vector2 &ln2, double *d);



public:
  Array<int> outl;
  Array<int> outlLns;

  Homography();
  Homography(unsigned nbPoints, unsigned nbLines=0);
  ~Homography();

  bool EstimateHom(double H[9], int &nbOutliers, double inlPcent=.8, bool returnOutl=false, int normalize=1, int NLrefine=HOMEST_SYM_XFER_ERROR, int verbose=1);
  bool EstimateAff(double H[9], int &nbOutliers, double inlPcent=.8, bool returnOutl=false, int normalize=1, int verbose=1);

  bool ComputeHomLS(double H[9], bool normalize=true);
  bool ComputeHomRobustLS(double H[9], bool normalize=true, double thrInlier=1., double thrAngle=1.);
  bool ComputeHom(double a1[2], double a2[2], double a3[2], double a4[2], 
                  double b1[2], double b2[2], double b3[2], double b4[2],
                  double H[9]);
  bool ComputeAff(double a1[2], double a2[2], double a3[2], 
                  double b1[2], double b2[2], double b3[2], 
                  double H[9]);
  
  inline void InsertNext(double pt1[2], double pt2[2]);
  inline void InsertNextPt(double pt1[2], double pt2[2]);
  inline void InsertNextLn(double ln1[2], double ln2[2]);
  inline void InsertNextLn(double a1[2], double b1[2], double a2[2], double b2[2]);
  inline void SetFirst();
  inline void Resize(unsigned nbPoints, unsigned nbLines=0);

  inline void LinePts2Para(double p1[2], double p2[2], double l[3]);
};


inline void MapPoint(double xin, double yin, double H[9], double &xout, double &yout);
inline void MapPoint(double in[2], double H[9], double out[2]);
inline void GetPosScaleAngle( double x, double y, double H[9], double &xr, double &yr, double &scale, double &angle);





/****************************** INLINE METHODES *********************************/
/**
 * Insert a point to the least squares homography matrix
 */
inline void Homography::InsertPointCoeff(Vector2 &pt1, Vector2 &pt2, double *d)
{
  double t1, t2, t3, t4;

  t1 = pt1.x;
  t2 = pt1.y;
  t3 = pt2.x;
  t4 = pt2.y;

  d[0] = -t1;
  d[1] = -t2;
  d[2] = -1.0;
  d[3] = 0.0;
  d[4] = 0.0;
  d[5] = 0.0;
  d[6] = t3*t1;
  d[7] = t2*t3;
  d[8] = t3;
  d[9] = 0.0;
  d[10] = 0.0;
  d[11] = 0.0;
  d[12] = d[0];
  d[13] = d[1];
  d[14] = -1.0;
  d[15] = t4*t1;
  d[16] = t4*t2;
  d[17] = t4;
}

/**
 * Insert a line to the least squares homography matrix
 */
inline void Homography::InsertLineCoeff(Vector2 &ln1, Vector2 &ln2, double *d)
{
  double t1, t2, t3, t4;

  t1 = ln1.x;  //x
  t2 = ln1.y;  //y
  t3 = ln2.x;  //u
  t4 = ln2.y;  //v

  d[0] = -t3;
  d[1] = 0.;
  d[2] = t3*t1;
  d[3] = -t4;
  d[4] = 0.;
  d[5] = t4*t1;
  d[6] = -1;
  d[7] = 0.;
  d[8] = t1;
  d[9] = 0.;
  d[10] = -t3;
  d[11] = t3*t2;
  d[12] = 0.;
  d[13] = -t4;
  d[14] = t4*t2;
  d[15] = 0.;
  d[16] = -1;
  d[17] = t2;
}

/**
 * allocate memory for points
 */
inline void Homography::Resize(unsigned nbPoints, unsigned nbLines)
{
  if (nbPoints>0)
  {
    pts1.Resize(nbPoints);
    pts2.Resize(nbPoints);
  }
  if (nbLines>0)
  {
    lns1.Resize(nbLines);
    lns2.Resize(nbLines);
  }
  lastPt=0;
  lastLn=0;
}

/**
 * set last pointer to first point
 */
inline void Homography::SetFirst()
{
  lastPt = 0;
  lastLn = 0;
}

/**
 * insert next point
 */
inline void Homography::InsertNext(double pt1[2], double pt2[2])
{
  if (lastPt<pts1.Size())
  {
    pts1[lastPt].x = pt1[0];
    pts1[lastPt].y = pt1[1];
    pts2[lastPt].x = pt2[0];
    pts2[lastPt].y = pt2[1];
    lastPt++;
  }
  else             //fall back solution if anyone did not allocate 
  {
    pts1.PushBack(Vector2(pt1[0],pt1[1]));
    pts2.PushBack(Vector2(pt2[0],pt2[1]));
    lastPt++;
  }
}

/**
 * insert next point
 */
inline void Homography::InsertNextPt(double pt1[2], double pt2[2])
{
  if (lastPt<pts1.Size())
  {
    pts1[lastPt].x = pt1[0];
    pts1[lastPt].y = pt1[1];
    pts2[lastPt].x = pt2[0];
    pts2[lastPt].y = pt2[1];
    lastPt++;
  }
  else             //fall back solution if anyone did not allocate 
  {
    pts1.PushBack(Vector2(pt1[0],pt1[1]));
    pts2.PushBack(Vector2(pt2[0],pt2[1]));
    lastPt++;
  }
}

/**
 * insert next line
 * definition of lines:
 *   ax + by + c = 1
 *          ( a )   ( a/c )
 *   -> l = | b | = | b/c |
 *          ( c )   (  1  )
 */
inline void Homography::InsertNextLn(double ln1[2], double ln2[2])
{
  if (lastLn<lns1.Size())
  {
    lns1[lastLn].x = ln1[0];
    lns1[lastLn].y = ln1[1];
    lns2[lastLn].x = ln2[0];
    lns2[lastLn].y = ln2[1];
    lastLn++;
  }
  else             //fall back solution if anyone did not allocate 
  {
    lns1.PushBack(Vector2(ln1[0],ln1[1]));
    lns2.PushBack(Vector2(ln2[0],ln2[1]));
    lastLn++;
  }
}

/**
 * insert line defined by two points [a1 b1] and [a2 b2]
 */
inline void Homography::InsertNextLn(double a1[2], double b1[2], double a2[2], double b2[2])
{
  double l[3];

  
  if (lastLn<lns1.Size())
  {
    LinePts2Para(a1,b1,l);
    lns1[lastLn].x = l[0]/l[2];
    lns1[lastLn].y = l[1]/l[2];
    LinePts2Para(a2,b2,l);
    lns2[lastLn].x = l[0]/l[2];
    lns2[lastLn].y = l[1]/l[2];
    lastLn++;
  }
  else             //fall back solution if anyone did not allocate 
  {
    LinePts2Para(a1,b1,l);
    lns1.PushBack(Vector2(l[0]/l[2],l[1]/l[2]));
    LinePts2Para(a2,b2,l);
    lns2.PushBack(Vector2(l[0]/l[2],l[1]/l[2]));
    lastLn++;
  }
}

/**
 * Compute parameters of a line defined by two points
 */
inline void Homography::LinePts2Para(double p1[2], double p2[2], double l[3])
{
  l[0] = p1[1]-p2[1];
  l[1] = p2[0]-p1[0];
  l[2] = p1[0]*p2[1] - p1[1]*p2[0];
}

/**
 * Map a point using an (affine) homography
 * @param H pointer to a 3x3 homography matrix
 */
inline void MapPoint(double xin, double yin, double H[9], double &xout, double &yout)
{
  xout = H[0]*xin + H[1]*yin + H[2];
  yout = H[3]*xin + H[4]*yin + H[5];
  double t = H[6]*xin + H[7]*yin + H[8];
  xout /= t;
  yout /= t;
}

/**
 * Map a point using an (affine) homography
 * @param H pointer to a 3x3 homography matrix
 */
inline void MapPoint(double in[2], double H[9], double out[2])
{
  out[0] = H[0]*in[0] + H[1]*in[1] + H[2];
  out[1] = H[3]*in[0] + H[4]*in[1] + H[5];
  double t = H[6]*in[0] + H[7]*in[1] + H[8];
  out[0] /= t;
  out[1] /= t;
}

/**
 * returns angle and scale from homography
 */
inline void GetPosScaleAngle( double x, double y, double H[9], double &xr, double &yr, double &scale, double &angle)
{
  double x2=x+10.;
  double y2=y;
  double x2H,y2H;
  MapPoint(x,y,H,xr,yr);
  MapPoint(x2,y2,H,x2H,y2H);

  double dxH=x2H-xr;
  double dyH=y2H-yr;
  scale=((sqrt(dxH*dxH+dyH*dyH))/10.);
  angle=atan2(dyH,dxH);
}



}

#endif

