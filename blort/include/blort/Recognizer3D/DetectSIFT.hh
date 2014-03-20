/**
 * Johann Prankl, 2010-01-27 
 * prankl@acin.tuwien.ac.at
 */

#ifndef P_DETECT_SIFT_HH
#define P_DETECT_SIFT_HH

#include <limits.h>
#ifndef WIN32
#include <wait.h>
#endif
#include <blort/Recognizer3D/PNamespace.hh>
#include <blort/Recognizer3D/KeypointDescriptor.hh>
#include <blort/Recognizer3D/Array.hh>


namespace P
{

typedef float SIFTDescriptor[128];



class DetectSIFT
{
private:

  void SavePGMImage(const char *filename, IplImage *grey);

public:
  DetectSIFT();
  ~DetectSIFT();

  void Operate(IplImage *img,Array<KeypointDescriptor*> &keys);
  void Draw(IplImage *img, Array<KeypointDescriptor*> &keys);
};


/************************** INLINE METHODES ******************************/



}

#endif

