/**
 * $Id: DetectGPUSIFT.cc 34111 2012-07-03 14:29:54Z student5 $
 * Johann Prankl, 2010-01-27 
 * prankl@acin.tuwien.ac.at
 */


#include <blort/Recognizer3D/DetectGPUSIFT.hh>
#include <blort/Recognizer3D/SDraw.hh>

namespace P 
{

DetectGPUSIFT::DetectGPUSIFT()
{

  //init sift
  //char * argv[] = {"-m", "-s", "-v", "1"};
  char * argv[] = {"-m", "-fo","-1", "-s", "-v", "0", "-pack"};
  //char * argv[] = {"-m", "-s", "-w", "3", "-fo", "-1", "-loweo"};
  //char * argv[] = {"-fo","-1","-v", "1"};

  int argc = sizeof(argv)/sizeof(char*);
  sift = new SiftGPU;
  sift->ParseParam(argc, argv);

  //create an OpenGL context for computation
  if(sift->CreateContextGL() != SiftGPU::SIFTGPU_FULL_SUPPORTED)
    throw Except(__HERE__,"SiftGPU is not fully supported");
}

DetectGPUSIFT::~DetectGPUSIFT()
{
  delete sift;
}




/************************************** PRIVATE ************************************/






/************************************** PUBLIC ************************************/

void DetectGPUSIFT::Operate(IplImage *img, Array<KeypointDescriptor*> &keys)
{
  for (unsigned i=0; i<keys.Size(); i++)
    delete keys[i];
  keys.Clear();

  if (img->depth != IPL_DEPTH_8U && img->nChannels!=1)
    throw Except(__HERE__,"Wrong image type!");

  if(sift->RunSIFT(img->width, img->height,(unsigned char *)img->imageData,GL_LUMINANCE,GL_UNSIGNED_BYTE))
  {
    KeypointDescriptor *k;

    int num = sift->GetFeatureNum();
    if (num>0)
    {
      Array<SiftGPU::SiftKeypoint> ks(num);
      Array<SIFTDescriptor> desc(num);

      sift->GetFeatureVector(&ks[0], (float*)&desc[0]);

      //copy sift
      for (unsigned i=0; i<desc.Size(); i++)
      {
        k = new KeypointDescriptor(KeypointDescriptor::DOG_SIFT, ks[i].x,ks[i].y,ks[i].s, -ks[i].o);
        k->AllocVec(128);
        CopyVec((float*)&desc[i], k->vec, 128);

        keys.PushBack(k);
      }
    }else cout<<"[DetectGPUSIFT::Operate] No SIFT found"<<endl;
  }else throw Except(__HERE__, "SiftGPU Error!");

}

/**
 * Match SIFT descriptor with codebook on gpu
 */
/*void DetectGPUSIFT::Match(Array<KeypointDescriptor*> &keys, Array<CodebookEntry *> &cb, int (*matches)[2], int buf_size, int &num)
{
  if (gpuMemSize < keys.Size()) matcher->SetMaxSift((int)keys.Size());
  if (gpuMemSize < cb.Size()) matcher->SetMaxSift((int)cb.Size());

  P::Array<SIFTDescriptor> desc1(cb.Size());
  P::Array<SIFTDescriptor> desc2(keys.Size());

  for (unsigned i=0; i<cb.Size(); i++)
    CopyVec(cb[i]->model->vec, (float*)&desc1, cb[i]->model->GetSize());  
  for (unsigned i=0; i<keys.Size(); i++)
    CopyVec(keys[i]->vec, (float*)&desc2, keys[i]->GetSize());  

  matcher->SetDescriptors(0, (int)desc1.Size(), (float*)&(desc1)[0]); //codebook
  matcher->SetDescriptors(1, (int)desc2.Size(), (float*)&(desc2)[0]); //keys

  num = matcher->GetSiftMatch(buf_size, matches);
}*/



/***
 * Draw tracks
 */
void DetectGPUSIFT::Draw(IplImage *img, Array<KeypointDescriptor*> &keys)
{
  for (unsigned i=0; i<keys.Size(); i++)
  {
      keys[i]->Draw(img,*keys[i],CV_RGB(255,0,0));
  }
}



}

