/**
 * Johann Prankl, 2010-01-27 
 * prankl@acin.tuwien.ac.at
 */

#ifndef P_COMPUTE_OBJECT_MODEL_3D_HH
#define P_COMPUTE_OBJECT_MODEL_3D_HH

#include <blort/Recognizer3D/PNamespace.hh>
#include <blort/Recognizer3D/Object3D.hh>
#include <blort/Recognizer3D/Array.hh>
#include <blort/Recognizer3D/Definitions.hh>
#include <blort/Recognizer3D/CodebookEntry.hh>
#include <blort/Recognizer3D/Geometry.hh>
#include <string>

namespace P
{

/**
 * Create a plane model
 * Map keypoints to zero centered coordinates and create codebook
 */
class ModelObject3D
{
private:
  void ComputeNewHnorm(P::Array<KeypointDescriptor*> &keys, double Hnorm[9]);
  void InsertMeanShift(Array<KeypointDescriptor* > &keys, P::Array<CodebookEntry*> &codebook, double H[9]);


public:
  ModelObject3D();
  ~ModelObject3D();

  void AddToModel(Array<KeypointDescriptor *> &keys, Object3D &obj);
  void SaveModel(const char *filename, Object3D &obj);
  bool LoadModel(const std::string filename, Object3D &obj);

  inline void MapPoint(double xin, double yin, double H[9], double &xout, double &yout);
  inline void MapPoint(double in[2], double H[9], double out[2]);
  inline void GetPosScaleAngle( double x, double y, double H[9], double &xr, double &yr, double &scale, double &angle);

};


/*********************** INLINE METHODES **************************/

/**
 * Map a point using an (affine) homography
 * @param H pointer to a 3x3 homography matrix
 */
inline void ModelObject3D::MapPoint(double xin, double yin, double H[9], double &xout, double &yout)
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
inline void ModelObject3D::MapPoint(double in[2], double H[9], double out[2])
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
inline void ModelObject3D::GetPosScaleAngle( double x, double y, double H[9], double &xr, double &yr, double &scale, double &angle)
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

