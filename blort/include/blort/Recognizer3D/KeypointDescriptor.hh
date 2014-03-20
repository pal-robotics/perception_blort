/**
 * Johann Prankl, 2010-01-27 
 * prankl@acin.tuwien.ac.at
 */

#ifndef P_KEYPOINT_DESCRIPTOR_HH
#define P_KEYPOINT_DESCRIPTOR_HH

#include <iostream>
#include <fstream>
#include <float.h>
//#include <opencv/cv.h>
//#include <opencv/cxcore.h>
#include <opencv2/opencv.hpp>
#include <blort/Recognizer3D/PNamespace.hh>
#include <blort/Recognizer3D/Keypoint.hh>
#include <blort/Recognizer3D/Array.hh>
#include <blort/Recognizer3D/Vector2.hh>
#include <blort/Recognizer3D/SDraw.hh>


//#define SAVE_PATCHES

#define PATCH_MASK_SIZE 16 


namespace P
{


//class ObjectModel;


class KeypointDescriptor : public Keypoint
{
public:
  enum Type
  {
    DOG_SIFT,
    LOWE_DOG_SIFT,
    MSER_SIFT,
    UVMSER_SIFT,
    HESLAP_SIFT,
    MAX_TYPE,
    UNDEF = MAX_TYPE
  };

  static float DOG_SIFT_THR;
  static float LOWE_DOG_SIFT_THR;
  static float MSER_SIFT_THR;
  static float COL_THR;
  static float HESLAP_SIFT_THR;


public:
  static const unsigned KEDE_DELETE;             //delete occurrence
  static const unsigned KEDE_INSERT;             //insert to codebook
  static const unsigned KEDE_DETECTED;           //keypoint detected flag

  unsigned flags;     //flags

  Type type;
  float *vec;
  unsigned size;

  CvMat *pos;
  unsigned cnt_pos;
  P::Vector2 p_rect;

  IplImage *mask;        //mask for segmentation
  IplImage *bgmask;
  IplImage *patch;       //normalised grey scale patch

  //some statistics....
  float reliability;     //mean number of detections
  unsigned cnt_rel;
  float mean_error;   //mean projection error
  float var_error;    //variance of projection error
  unsigned cnt_err;             //number of possible detections

  unsigned occNum;             //number of occurrences within codebook ...
                               //... or number of matches of image keypoints
  unsigned chainId;
  unsigned chainCnt;

  KeypointDescriptor();
  KeypointDescriptor(Type t);
  KeypointDescriptor(KeypointDescriptor *k);
  KeypointDescriptor(Type t, double x, double y, float s, float a);
  KeypointDescriptor(Type t, double x,double y,float s,float a,
                     float _m11,float _m12,float _m21,float _m22);
  ~KeypointDescriptor();

  static const char* TypeName(Type t);
  static Type EnumType(const char *type_name);
  bool GetVoteCenter(KeypointDescriptor *current, Vector2 &v, 
                     double &delta_angle, double &delta_scale);
  void SaveMask(IplImage *img);
  void ProjectMask(KeypointDescriptor *model,IplImage *img, float w);
  void ProjectBgMask(KeypointDescriptor *model,IplImage *img, float w);
  void ProjectOccl(P::Vector2 &center, float scale, float angle, 
                   IplImage *img, float w);
  void SavePatch(IplImage *img);
  void ProjectPatch(KeypointDescriptor *model,IplImage *img);
  float Match(KeypointDescriptor *k);
  void ReadKeypoint( ifstream &in );
  void ReadDescriptor( ifstream &in, unsigned size_in);
  void WriteKeypoint( ofstream &out );
  void WriteDescriptor(ofstream &out );

  void SetReliability(float f);
  void SetError(float e); 
  void Copy(KeypointDescriptor *k);
  void AllocVec(unsigned s);
  void CopyVec(KeypointDescriptor *k);
  inline float* GetVec(){return vec;}
  inline float GetVec(unsigned i){return (i<size?vec[i]:0);}
  inline Type GetType(){return type;}
  inline unsigned GetSize(){return size;} 
  inline bool Have3D(){return (pos!=0?true:false);}
  float DistSqr(float *desc1, float *desc2, unsigned cnt);
  void Add(float *desc);
  void Div(float num);
  void Mul(float num);
  void SetZero();
  void MatchSift(KeypointDescriptor *k, float thr, float &dist); 
  inline float GetThr();
  inline void SetPos(float x, float y, float z);
 
  static void Draw(IplImage *img, KeypointDescriptor &k, CvScalar col);  
  static void LoadAll(ifstream &is, KeypointDescriptor &k);
  static void SaveAll(ofstream &os, const KeypointDescriptor &k);
};


void CopyVec(float *svec, float *dvec, unsigned size);
void WriteKeypoints(P::Array<KeypointDescriptor*> &ks, const char* file, int format=0);
void LoadKeypoints(const char* file, P::Array<KeypointDescriptor*> &ks, int format=0);
void LoadLoweKeypoints(const char* file, P::Array<Keypoint*> &ks, int format=0);
void LoadLoweKeypoints(const char* file, P::Array<KeypointDescriptor*> &ks, int format=0);






/*************************** INLINE METHODES **************************/
inline void KeypointDescriptor::AllocVec(unsigned s)
{
  if (vec!=0) delete[] vec;
  size=s;
  vec = new float[size];
}

inline void KeypointDescriptor::CopyVec(KeypointDescriptor *k)
{
  if (k->vec!=0){
    AllocVec(k->size);
    for (unsigned i=0; i<size; i++){
      vec[i]=k->vec[i];
    }
  }
}

inline void KeypointDescriptor::Copy(KeypointDescriptor *k)
{
  p=k->p;
  if (k->Have3D())
  {
    if (!Have3D()) pos = cvCreateMat(3,1, CV_32F);
    cvCopy(k->pos,pos);
  }
  p_rect=k->p_rect;
  cnt_pos=k->cnt_pos;
  scale=k->scale;
  angle=k->angle;

  mi11=k->mi11;
  mi12=k->mi12;
  mi21=k->mi21;
  mi22=k->mi22;
  
  chainId=k->chainId;
  chainCnt=k->chainCnt;

  type=k->type;
  CopyVec(k);
  
  if (k->mask!=0){ 
    if (mask==0) mask=cvCreateImage( cvSize(PATCH_MASK_SIZE,PATCH_MASK_SIZE), IPL_DEPTH_32F, 1 );
    cvCopy(k->mask, mask);
  }
  if (k->bgmask!=0){ 
    if (bgmask==0) bgmask=cvCreateImage( cvSize(PATCH_MASK_SIZE,PATCH_MASK_SIZE), IPL_DEPTH_32F, 1 );
    cvCopy(k->bgmask, bgmask);
  }

  if (k->patch!=0){ 
    if (patch==0) patch=cvCreateImage( cvSize(PATCH_MASK_SIZE,PATCH_MASK_SIZE), IPL_DEPTH_8U, 1 );
    cvCopy(k->patch, patch);
  }
  cnt_err=k->cnt_err;
  flags=k->flags;
  cnt_rel=k->cnt_rel;
  reliability=k->reliability;
  mean_error=k->mean_error;
  var_error=k->var_error;
  occNum=k->occNum;
}

/**
 * Return squared distance between two keypoint descriptors.
 */
inline float KeypointDescriptor::DistSqr(float *desc1, float *desc2, unsigned cnt)
{
    register unsigned i;
    float dif;
    float distsq = 0;

    for (i = 0; i < cnt; i++) {
      dif = *desc1++ - *desc2++;
      distsq += dif * dif;
    }
    return distsq;
}

/**
 * Add two descriptors
 */
inline void KeypointDescriptor::Add(float *desc)
{
  register unsigned i;
  register float *d = GetVec();

  for (i = 0; i < GetSize(); i++) {
    *d++ += *desc++;
  }
}

/**
 * Set a descriptor to 0
 */
inline void KeypointDescriptor::SetZero()
{
  register float *d=GetVec();
  register unsigned z;

  for (z=0; z< GetSize(); z++){
    *d++ = 0.;
  }
}

/**
 * Devide a descriptor by ..
 */
inline void KeypointDescriptor::Div(float num)
{
  register float *d=GetVec();
  register unsigned z;

  for (z=0; z< GetSize(); z++){
    *d++ /= num;
  }
}

/**
 * Multiply a descriptor with ..
 */
inline void KeypointDescriptor::Mul(float num)
{
  register float *d=GetVec();
  register unsigned z;

  for (z=0; z< GetSize(); z++){
    *d++ *= num;
  }
}


/**
 * MatchSift
 * tests the euclidean distance of the sift descriptor
 */
inline void KeypointDescriptor::MatchSift(KeypointDescriptor *k, float thr, float &dist)
{
  dist = DistSqr(vec, k->vec, GetSize());
  if (dist>=thr) dist=FLT_MAX;
}

/**
 * compute statistics of keypoint detection (mean number of detections of a keypoint
 * in an image sequence
 */
inline void KeypointDescriptor::SetReliability(float f)
{
  reliability = ((float)(cnt_rel*reliability + f))/((float)(cnt_rel+1));
  cnt_rel++;
}

/**
 * set mean error and variance
 */
inline void KeypointDescriptor::SetError(float e)
{
  mean_error = ((float)cnt_err*mean_error + e)/((float)cnt_err+1.);
 
  var_error = 1./((float)cnt_err+1.) * ((float)cnt_err*var_error + 
                                    (float)cnt_err/((float)cnt_err+1.)*Sqr(mean_error - e));
  cnt_err++;
}

inline float KeypointDescriptor::GetThr()
{
  switch(GetType())
  {
    case DOG_SIFT:
      return DOG_SIFT_THR;
      break;
    case LOWE_DOG_SIFT:
      return LOWE_DOG_SIFT_THR;
      break;
    case MSER_SIFT:
      return MSER_SIFT_THR;
      break;
    case UVMSER_SIFT:
      return MSER_SIFT_THR;
      break;
    case HESLAP_SIFT:
      return HESLAP_SIFT_THR;
      break;
    default:
      return FLT_MAX;
      break;        
  }
  return FLT_MAX;
}

inline void KeypointDescriptor::SetPos(float x, float y, float z)
{
  if (!Have3D()) pos = cvCreateMat(3,1, CV_32F);

  pos->data.fl[0] = x;
  pos->data.fl[1] = y;
  pos->data.fl[2] = z;
}

/**
 * only copies a vector of floats
 */
inline void CopyVec(float *svec, float *dvec, unsigned size)
{
  for (register unsigned i=0; i<size; i++){
    *dvec++=*svec++;
  }
}

inline float MatchKeypoint(KeypointDescriptor *k1, KeypointDescriptor *k2)
{
  if (k1->GetType() != k2->GetType())
    return FLT_MAX;
  return k1->DistSqr(k1->vec, k2->vec, k1->GetSize()); 
}


} //--END--

#endif

