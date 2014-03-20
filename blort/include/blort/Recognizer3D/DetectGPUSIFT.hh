/**
 * Johann Prankl, 2010-01-27 
 * prankl@acin.tuwien.ac.at
 */

#ifndef P_DETECT_GPUSIFT_HH
#define P_DETECT_GPUSIFT_HH

#include <limits.h>
#include <GL/glut.h>
#include <SiftGPU.h>

#include <blort/Recognizer3D/PNamespace.hh>
#include <blort/Recognizer3D/KeypointDescriptor.hh>
#include <blort/Recognizer3D/CodebookEntry.hh>
#include <blort/Recognizer3D/Array.hh>


#ifndef WIN32
#include <dlfcn.h>
#endif


namespace P
{

typedef float SIFTDescriptor[128];


class DetectGPUSIFT
{
private:
  SiftGPU *sift;
  
public:
  DetectGPUSIFT();
  ~DetectGPUSIFT();

  void Operate(IplImage *img, Array<KeypointDescriptor*> &keys);
  //void Match(Array<KeypointDescriptor*> &keys, Array<CodebookEntry *> &cb, int (*matches)[2], int buf_size, int &num); 

  void Draw(IplImage *img, Array<KeypointDescriptor*> &keys);
};


/************************** INLINE METHODES ******************************/



}

#endif

