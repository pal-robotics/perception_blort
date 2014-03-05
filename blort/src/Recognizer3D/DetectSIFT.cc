/**
 * $Id: DetectSIFT.cc 34111 2012-07-03 14:29:54Z student5 $
 * Johann Prankl, 2010-01-27 
 * prankl@acin.tuwien.ac.at
 */

#include <errno.h>
#include <blort/Recognizer3D/DetectSIFT.hh>
#include <blort/Recognizer3D/SDraw.hh>
#include <string>
#include <unistd.h>

namespace P 
{

    DetectSIFT::DetectSIFT()
    {
    }

    DetectSIFT::~DetectSIFT()
    {
    }




    /************************************** PRIVATE ************************************/
    /**
 * SaveImage
 */
    void DetectSIFT::SavePGMImage(const char *filename, IplImage *grey)
    {
        if (grey->nChannels!=1 && grey->depth != IPL_DEPTH_8U)
            throw Except(__HERE__,"Wrong image format!");

        unsigned storage_size = grey->width*grey->height*grey->depth;

        FILE *file = fopen(filename, "w");
        if(file == NULL)
            throw Except(__HERE__, "failed to open file %s:", filename,strerror(errno));
        fprintf(file,"P5\n%u %u\n255\n", grey->width, grey->height);

        fwrite(grey->imageData, 1, storage_size, file);

        fclose(file);
    }










    /************************************** PUBLIC ************************************/

    void DetectSIFT::Operate(IplImage *img, Array<KeypointDescriptor*> &keys)
    {
#ifndef WIN32 // TODO TODO TODO> DIRTY HACK, cause wait.h not defined in windoof
        if (img->depth != IPL_DEPTH_8U && img->nChannels!=1)
            throw Except(__HERE__,"Wrong image type!");

        SavePGMImage("./detbin/grey.pgm", img);

        int pid,status;
        switch(pid = fork()){
        case 0:
            execl(std::string("/bin/sh").c_str(),
                  std::string("/bin/sh").c_str(),
                  std::string("-c").c_str(),
                  std::string("./detbin/sift <./detbin/grey.pgm >./detbin/grey.key").c_str(),
                  (char*)0);
            break;

        default: while(wait(&status) != pid);
        }

        LoadLoweKeypoints("./detbin/grey.key", keys, 0);

        //..or only correct type
        for (unsigned i=0; i<keys.Size(); i++)
            ((KeypointDescriptor*)keys[i])->type = KeypointDescriptor::LOWE_DOG_SIFT;
#else
	printf("[Recognizer3D DetectSIFT::Operate] not implemented for WIN32\n");
#endif
    }


    /***
 * Draw tracks
 */
    void DetectSIFT::Draw(IplImage *img, Array<KeypointDescriptor*> &keys)
    {
        for (unsigned i=0; i<keys.Size(); i++)
        {
            keys[i]->Draw(img,*keys[i],CV_RGB(255,0,0));
        }
    }



}

