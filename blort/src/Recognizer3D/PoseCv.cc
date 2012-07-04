/**
 * $Id: PoseCv.cc 34111 2012-07-03 14:29:54Z student5 $
 * Johann Prankl, 2010-01-27 
 * prankl@acin.tuwien.ac.at
 */


#include <blort/Recognizer3D/PoseCv.hh>


namespace P 
{



/***************************** PoseCv ******************************
 * Constructor/Destructor
 */
PoseCv::PoseCv()
{
  R = cvCreateMat( 3, 3, CV_32F );
  t = cvCreateMat( 3, 1, CV_32F );
  n = cvCreateMat( 3, 1, CV_32F );
}

//BENCE
PoseCv::PoseCv(cv::Mat translation, cv::Mat rotation)
{
    R = cvCreateMat( 3, 3, CV_32F );
    t = cvCreateMat( 3, 1, CV_32F );
    n = cvCreateMat( 3, 1, CV_32F );

    for(int i=0; i<rotation.rows; ++i)
        for(int j=0; j<rotation.cols; ++j)
            cvmSet(R,i,j,rotation.at<float>(i,j));

    for(int i=0; i<translation.rows; ++i)
        for(int j=0; j<translation.cols; ++j)
            cvmSet(t,i,j,translation.at<float>(i,j));

    cvmSet(n,0,0,1);
    cvmSet(n,1,0,0);
    cvmSet(n,2,0,0);
}

PoseCv::~PoseCv()
{
  cvReleaseMat(&R);
  cvReleaseMat(&t);
  cvReleaseMat(&n);
}

/**************************** PRIVATE *************************************/



/**************************** PUBLIC *************************************/
void InitializePoseCv(PoseCv &pose)
{
  cvmSet(pose.R,0,0,1); cvmSet(pose.R,0,1,0); cvmSet(pose.R,0,2,0);
  cvmSet(pose.R,1,0,0); cvmSet(pose.R,1,1,1); cvmSet(pose.R,1,2,0);
  cvmSet(pose.R,2,0,0); cvmSet(pose.R,2,1,0); cvmSet(pose.R,2,2,1);

  cvmSet(pose.t,0,0,0);
  cvmSet(pose.t,1,0,0);
  cvmSet(pose.t,2,0,0);

  cvmSet(pose.n,0,0,1);
  cvmSet(pose.n,1,0,0);
  cvmSet(pose.n,2,0,0);

   
}

void DeletePoseCv(Array<PoseCv*> &ps)
{
  for (unsigned i=0; i<ps.Size(); i++)
    delete ps[i];
  ps.Clear();
}


} // --THE END--



