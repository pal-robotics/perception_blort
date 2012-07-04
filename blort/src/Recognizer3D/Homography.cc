/**
 * $Id: Homography.cc 34111 2012-07-03 14:29:54Z student5 $
 * Johann Prankl, 2010-03-30
 * prankl@acin.tuwien.ac.at
 */


#include <blort/Recognizer3D/Homography.hh>


#ifndef M_SQRT2
#define M_SQRT2  1.41421356237309504880  // sqrt(2) 
#endif

//#define DEBUG
#define HOM_RANSAC_ETA0 0.01 //0.01
#define HOM_MAX_RANSAC_TRIALS 10000

namespace P 
{

Homography::Homography()
 : lastPt(0), lastLn(0), T1(0), invT(0)
{
}

Homography::Homography(unsigned nbPoints, unsigned nbLines)
 : lastPt(0), lastLn(0), T1(0), invT(0)
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
}

Homography::~Homography()
{
  if (T1!=0) delete[] T1;
  if (invT!=0) delete[] invT;
}

/************************************* PRIVATE ***************************************/
/**
 * GetRandIdx
 */
void Homography::GetRandIdx(unsigned size, unsigned num, P::Array<unsigned> &idx)
{
  unsigned temp;
  idx.Clear();
  for (unsigned i=0; i<num; i++)
  {
    do{
      temp = rand()%size;
    }while(idx.Contains(temp));
    idx.PushBack(temp);
  }
}

/**
 * Count point inlier
 */
void Homography::CountInlierPts(double H[9], double thr, int &inl)
{
  Vector2 p;
 
  for (unsigned i=0; i<lastPt; i++)
  {
    MapPoint(&pts1[i].x, H, &p.x);

    if (Distance(p,pts2[i]) < thr)
      inl++;
  }
}

/**
 * Count point inlier
 */
void Homography::CountInlierLns(double H[9], double thr, double thrAngle, int &inl)
{
  double HT[9];
  Transpose33(H, HT);

  double thrAng = cos(thrAngle * M_PI/180.);         //for lines: inl = alpha [°]

  double k1, d1, k2, d2, dist1, dist2;
  Vector2 l2, dir1, dir2;
  Vector2 p1, p2, p0 = Vector2(0.,0.);

  for (unsigned i=0; i<lastLn; i++)
  {
    MapPoint(&lns2[i].x, HT, &l2.x);

    k1 = -lns1[i].x/lns1[i].y;
    d1 = -1./lns1[i].y;
    dir1.y = d1;
    dir1.x = 1.;
    dir1.Normalise();

    k2 = -l2.x/l2.y;
    d2 = -1./l2.y;
    dir2.y = d2;
    dir2.x = 1.;
    dir2.Normalise();

    p1.x = 1.;
    p1.y = k1+d1;
    p2.x = 1.;
    p2.y = k2+d2;

    dist1 = AbsDistPointToLine(p0, p1, dir1);
    dist2 = AbsDistPointToLine(p0, p2, dir2);

    if (fabs(dist1-dist2) < thr)
    {
      if (fabs(Dot(dir1,dir2)) >  thrAng)
      {
        inl++;
      }
    }
  }

}

/**
 * set least squares deata matrix (Ai*h = 0)
 */
void Homography::SetDataMatrix(double *d, double H[9], double thr, double thrAngle, int &numInl)
{
  numInl=0;

  //set inlier points
  Vector2 p;

  outl.Resize(lastPt);
  outl.Set(0);
   
  for (unsigned i=0; i<lastPt; i++)
  {
    MapPoint(&pts1[i].x, H, &p.x);

    if (Distance(p,pts2[i]) < thr)
    {
      InsertPointCoeff(pts1[i], pts2[i], &d[numInl*18]);
      numInl++;
    }
    else
    {
       outl[i] = 1;
    }
  }

  //set inlier lines
  double HT[9];
  Transpose33(H, HT);

  double thrAng = cos(thrAngle * M_PI/180.);         //for lines: inl = alpha [°]

  double k1, d1, k2, d2, dist1, dist2;
  Vector2 l2, dir1, dir2;
  Vector2 p1, p2, p0 = Vector2(0.,0.);

  outlLns.Resize(lastLn);
  outlLns.Set(0);
  for (unsigned i=0; i<lastLn; i++)
  {
    MapPoint(&lns2[i].x, HT, &l2.x);

    k1 = -lns1[i].x/lns1[i].y;
    d1 = -1./lns1[i].y;
    dir1.y = d1;
    dir1.x = 1.;
    dir1.Normalise();

    k2 = -l2.x/l2.y;
    d2 = -1./l2.y;
    dir2.y = d1;
    dir2.x = 1.;
    dir2.Normalise();

    p1.x = 1.;
    p1.y = k1+d1;
    p2.x = 1.;
    p2.y = k2+d2;

    dist1 = AbsDistPointToLine(p0, p1, dir1);
    dist2 = AbsDistPointToLine(p0, p2, dir2);

    if (fabs(dist1-dist2) < thr)
    {
      if (fabs(Dot(dir1, dir2)) >  thrAng)
      {
        InsertLineCoeff(lns1[i], lns2[i], &d[numInl*18]);
        numInl++;
      }
      else
        outlLns[i] = 1;
    }
    else
      outlLns[i] = 1;
  }

}

/**
 * Uses SVD to solve the matrix
 */
bool Homography::SolveHomography(CvMat *A, CvMat *W, CvMat *Ut, CvMat *Vt, double H[9])
{
  cvSVD( A, W, Ut, Vt, CV_SVD_MODIFY_A);

  // determine A's rank
  unsigned z;
  for(z=0; z<9; z++)
    if(IsZero(cvmGet(W,z,z))) break; // remaining singular values are smaller than this 

  if(z<8)
    return false; //A should have rank n-1

  // last is smallest singular value
  for (unsigned i=0; i<9; i++)
    H[i] = cvmGet(Vt,i,8);

  return true;
}

/**
 * Normalize points suggested by Hartley
 */
void Homography::NormalizePts(Array<Vector2> &in, unsigned numPts, Array<Vector2> &out, double T[9])
{
  if (numPts==0)
    return;

  double centx=0., centy=0.;
  double dist, scale;

  for(unsigned i=0; i<numPts; i++)
  {
    centx+=in[i].x;
    centy+=in[i].y;
  }

  centx/=(double)(numPts);
  centy/=(double)(numPts);

  out.Resize(numPts);
  for(unsigned i=0; i<numPts; i++)
  {
    out[i].x = in[i].x-centx;
    out[i].y = in[i].y-centy;
  }

  dist=0.0;
  for(unsigned i=0; i<numPts; i++)
    dist += Length(out[i]);

  dist/=(double)(numPts);
  scale = M_SQRT2/dist;

  for (unsigned i=0; i<numPts; i++)
    out[i]*=scale;

  T[0]=scale; T[1]=0.0;   T[2]=-centx*scale;
  T[3]=0.0;   T[4]=scale; T[5]=-centy*scale;
  T[6]=0.0;   T[7]=0.0;   T[8]=1.0;
}


/**
 * Normalize lines (see Dubrofsky, 2008)
 */
void Homography::NormalizeLns(double T[9], Array<Vector2> &in, unsigned numLns, Array<Vector2> &out)
{
  if (numLns==0)
    return;

  double t;
  out.Resize(numLns);
 
  for (unsigned i=0; i<numLns; i++)
  {
    t = 1./(T[0] + T[2]*in[i].x + T[5]*in[i].y);
    out[i].x = in[i].x*t;
    out[i].y = in[i].y*t;
  }
}

/**
 * Construct data matrix
 */
void Homography::SetDataMatrixPts(Array<Vector2> &pts1, Array<Vector2> &pts2, unsigned numPts, unsigned start, CvMat *A)
{
  if (numPts==0)
    return;

  double t1, t2, t3, t4;
  double *d = A->data.db + (start*2)*A->cols;  

  for (unsigned i=0; i<numPts; i++,d+=18)
  {
    t1 = pts1[i].x;
    t2 = pts1[i].y;
    t3 = pts2[i].x;
    t4 = pts2[i].y;

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
}

/**
 * Construct data matrix
 */
void Homography::SetDataMatrixLns(Array<Vector2> &lns1, Array<Vector2> &lns2, unsigned numLns, unsigned start, CvMat *A)
{
  if (numLns==0)
    return;

  double t1, t2, t3, t4;
  double *d = A->data.db + (start*2)*A->cols;  

  for (unsigned i=0; i<numLns; i++,d+=18)
  {
    t1 = lns1[i].x;  //x
    t2 = lns1[i].y;  //y
    t3 = lns2[i].x;  //u
    t4 = lns2[i].y;  //v

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
}

/**
 * Normalize H
 * nH = T2 * H * T1^-1
 */
void Homography::NormalizeH(double H[9], double T1[9], double T2[9], double nH[9])
{
  double T1_1[9], tmp[9];

  Inv33(T1, T1_1);
  Mul33(T2, H, tmp);     // tmp = T2 * H
  Mul33(tmp, T1_1, nH);  // nH = T2 * H * T1^-1
}

/**
 * Denormalize H
 * H = T2^-1 * nH * T1
 */
void Homography::DenormalizeH(double nH[9], double T1[9], double T2[9], double H[9])
{
  double T2_1[9], tmp[9];

  Inv33(T2, T2_1);
  Mul33(T2_1, nH, tmp);    // tmp = T2^-1 * nH 
  Mul33(tmp, T1, H);       // H = T2^-1 * nH * T1 
}









/************************************* PUBLIC ****************************************/


/**
 * Compute homography matrix robust to outlier
 * - least median of squares
 * - non linear refinement (lev mar)
 */
bool Homography::EstimateHom(double H[9], int &nbOutliers, double inlPcent, bool returnOutl, int normalize, int NLrefine, int verbose)
{
  int err;

  if (!returnOutl)
  {
    err = homest((double (*)[2])(&pts1[0]), (double (*)[2])(&pts2[0]), lastPt, inlPcent, H,
              normalize, NLrefine, 0, &nbOutliers, verbose);
  }
  else
  {
    Array<int> tmpOutl(pts1.Size());
    err = homest((double (*)[2])(&pts1[0]), (double (*)[2])(&pts2[0]), lastPt, inlPcent, H,
              normalize, NLrefine, &tmpOutl[0], &nbOutliers, verbose);

    outl.Resize(pts1.Size());
    outl.Set(0);
    for (int i=0; i<nbOutliers; i++)
    {
      outl[tmpOutl[i]] = 1;
    }
  }

  if (err==HOMEST_ERR)
    return false;
  return true;
}

/**
 * Compute affine homography matrix robust to outlier
 * - least median of squares
 */
bool Homography::EstimateAff(double H[9], int &nbOutliers, double inlPcent, bool returnOutl,  int normalize, int verbose)
{
  int err;

  if (!returnOutl)
  {
    err = homestaff((double (*)[2])(&pts1[0]), (double (*)[2])(&pts2[0]), lastPt, inlPcent, H,
                    normalize, 0, &nbOutliers, verbose);
  }
  else
  {
    outl.Resize(pts1.Size());
    err = homestaff((double (*)[2])(&pts1[0]), (double (*)[2])(&pts2[0]), lastPt, inlPcent, H,
                        normalize, &outl[0], &nbOutliers, verbose);
  }

  if (err==HOMEST_ERR)
    return false;
  return true;
}

/**
 * Compute projective mapping of four points
 */
bool Homography::ComputeHom(double a1[2], double a2[2], double a3[2], double a4[2], 
                            double b1[2], double b2[2], double b3[2], double b4[2],
                            double H[9])
{
  //double T1[9], invT[9];
  
  if (T1==0) T1 = new double[9];
  if (invT==0) invT = new double[9];

  // compute mapping of four (a) points to a unit square
  double s = (a2[0]-a3[0]) * (a4[1]-a3[1]) - (a4[0]-a3[0]) * (a2[1]-a3[1]);

  if (IsZero(s)) return false;

  T1[6] = ((a1[0]-a2[0]+a3[0]-a4[0])*(a4[1]-a3[1]) - 
           (a1[1]-a2[1]+a3[1]-a4[1])*(a4[0]-a3[0])) / s;
  T1[7] = ((a1[1]-a2[1]+a3[1]-a4[1])*(a2[0]-a3[0]) - 
           (a1[0]-a2[0]+a3[0]-a4[0])*(a2[1]-a3[1])) / s;
  T1[8] = 1.;

  T1[0] = a2[0]-a1[0] + T1[6]*a2[0];  T1[1] = a4[0]-a1[0] + T1[7]*a4[0];  T1[2] = a1[0];
  T1[3] = a2[1]-a1[1] + T1[6]*a2[1];  T1[4] = a4[1]-a1[1] + T1[7]*a4[1];  T1[5] = a1[1];

  if (!Inv33(T1, invT)) return false;

  // compute mapping of a unit square to four points
  s = (b2[0]-b3[0]) * (b4[1]-b3[1]) - (b4[0]-b3[0]) * (b2[1]-b3[1]);
  
  if (IsZero(s)) return false;

  T1[6] = ((b1[0]-b2[0]+b3[0]-b4[0])*(b4[1]-b3[1]) -
           (b1[1]-b2[1]+b3[1]-b4[1])*(b4[0]-b3[0])) / s;
  T1[7] = ((b1[1]-b2[1]+b3[1]-b4[1])*(b2[0]-b3[0]) -
           (b1[0]-b2[0]+b3[0]-b4[0])*(b2[1]-b3[1])) / s;
  T1[8] = 1.;

  T1[0] = b2[0]-b1[0] + T1[6]*b2[0];  T1[1] = b4[0]-b1[0] + T1[7]*b4[0];  T1[2] = b1[0];
  T1[3] = b2[1]-b1[1] + T1[6]*b2[1];  T1[4] = b4[1]-b1[1] + T1[7]*b4[1];  T1[5] = b1[1];
  
  // compute mapping four points (a) to four points (b)
  Mul33(T1,invT,H); 

  if (IsZero(H[8]))
    return false;

  Mul33(H, 1./H[8], H);

  return true;
}

/**
 * Compute affine mapping of three points
 */
bool Homography::ComputeAff(double a1[2], double a2[2], double a3[2], 
                            double b1[2], double b2[2], double b3[2], 
                            double H[9])
{
    double S;

    S = a1[0]*(a3[1]-a2[1]) + 
        a2[0]*(a1[1]-a3[1]) +
        a3[0]*(a2[1]-a1[1]);

    if (IsZero(S)) return false;

      S=1/S;

      //calculation of the affine parameter
      H[0] = S * (a1[1]*(b2[0]-b3[0]) + a2[1]*(b3[0]-b1[0])+ a3[1]*(b1[0]-b2[0]));
      H[1] = S * (a1[0]*(b3[0]-b2[0]) + a2[0]*(b1[0]-b3[0])+ a3[0]*(b2[0]-b1[0]));
      H[3] = S * (a1[1]*(b2[1]-b3[1]) + a2[1]*(b3[1]-b1[1])+ a3[1]*(b1[1]-b2[1]));
      H[4] = S * (a1[0]*(b3[1]-b2[1]) + a2[0]*(b1[1]-b3[1])+ a3[0]*(b2[1]-b1[1]));
      H[2] = S * (a1[0]*(a3[1]*b2[0]-a2[1]*b3[0])+
                  a2[0]*(a1[1]*b3[0]-a3[1]*b1[0])+
                  a3[0]*(a2[1]*b1[0]-a1[1]*b2[0]));
      H[5] = S * (a1[0]*(a3[1]*b2[1]-a2[1]*b3[1])+
                  a2[0]*(a1[1]*b3[1]-a3[1]*b1[1])+
                  a3[0]*(a2[1]*b1[1]-a1[1]*b2[1]));
      H[6] = 0;
      H[7] = 0;
      H[8] = 1;

  return true;
}


/**
 * Compute the least squares homography matrix using points and lines
 * This implementation needs points (for normalisation) and also uses lines for least squares
 */
bool Homography::ComputeHomLS(double H[9], bool normalize)
{
  if ((normalize && lastPt<2) || lastPt+lastLn < 5)
    return false;

  bool ok;
  unsigned size = 2*(lastPt+lastLn);

  CvMat *A = cvCreateMat(size, 9, CV_64F);
  CvMat *W = cvCreateMat(9, 9, CV_64F);
  CvMat *Ut = cvCreateMat(size,9, CV_64F);
  CvMat *Vt = cvCreateMat(9, 9, CV_64F);
  double T1[9], T2[9];

  if (normalize)
  {
    Array<Vector2> npts1, npts2;
    Array<Vector2> nlns1, nlns2;

    NormalizePts(pts1, lastPt, npts1, T1);
    NormalizePts(pts2, lastPt, npts2, T2);
    NormalizeLns(T1, lns1, lastLn, nlns1);
    NormalizeLns(T2, lns2, lastLn, nlns2);

    SetDataMatrixPts(npts1,npts2, lastPt, 0, A);
    SetDataMatrixLns(nlns1,nlns2, lastLn, lastPt, A);
  }
  else
  {
    SetDataMatrixPts(pts1, pts2, lastPt, 0, A);
    SetDataMatrixLns(lns1, lns2, lastLn, lastPt, A);
  }

  ok = SolveHomography(A, W, Ut, Vt, H);

  if (ok)
  {
    if (normalize)
      DenormalizeH(H, T1, T2, H);

    Mul33(H,1./H[8],H);
  }

  cvReleaseMat(&A);
  cvReleaseMat(&W);
  cvReleaseMat(&Ut);
  cvReleaseMat(&Vt);

  return ok;
}

/**
 * Compute the least squares homography matrix using points and lines
 * This implementation needs points (for normalisation) and also uses lines for least squares
 */
bool Homography::ComputeHomRobustLS(double H[9], bool normalize, double thrInlier, double thrAngle)
{
  if ((normalize && lastPt<2) || lastPt+lastLn < 5)
    return false;

  //init
  bool ok;
  unsigned size = 2*(lastPt+lastLn);

  double dA[size*9];
  double dW[9*9];
  double dUt[size*9];
  double dVt[9*9];
  double T1[9], T2[9];

  double thr;

  CvMat A, W, Ut, Vt;
  cvInitMatHeader(&A, 10, 9, CV_64F, &dA);
  cvInitMatHeader(&W, 9, 9, CV_64F, &dW);
  cvInitMatHeader(&Ut, 10, 9, CV_64F, &dUt);
  cvInitMatHeader(&Vt, 9, 9, CV_64F, &dVt);

  if (normalize)
  {
    NormalizePts(pts1, lastPt, pts1, T1);
    NormalizePts(pts2, lastPt, pts2, T2);
    NormalizeLns(T1, lns1, lastLn, lns1);
    NormalizeLns(T2, lns2, lastLn, lns2);
    thr = T2[0]; 
  }
  else
  {
    thr = thrInlier;
  }

  //ransac
  int numLines, k=0, num = lastPt+lastLn;
  double eps = 4./(double)num;
  int inl, inls = 0;
  Array<unsigned> idx;
  double Ht[9];
  srand(time(NULL));

  while (pow(1. - pow(eps,4),k) >= HOM_RANSAC_ETA0 && k<HOM_MAX_RANSAC_TRIALS)
  {
    do{
      GetRandIdx(num, 5, idx);

      numLines=0;
      for (unsigned i=0; i<5; i++)
        if (idx[i]<lastPt)
          numLines++;

    }while(numLines < 2);

    for (unsigned i=0; i<5; i++)
    {
      if (idx[i]<lastPt)
        InsertPointCoeff(pts1[idx[i]], pts2[idx[i]], &dA[i*18]);
      else
        InsertLineCoeff(lns1[idx[i]-lastPt], lns2[idx[i]-lastPt], &dA[i*18]);
    }

    if (SolveHomography(&A, &W, &Ut, &Vt, Ht))
    {
      inl=0;
      CountInlierPts(Ht, thr, inl);
      CountInlierLns(Ht, thr,thrAngle, inl);

      if (inl > inls)
      {
        inls = inl;
        eps = (double)inls / (double)num;
        for (unsigned i=0; i<9; i++)
          H[i] = Ht[i];
      }
    }

    k++;
  }
  #ifdef DEBUG
  cout<<"ComputeHomRobustLS: Number of RANSAC trials: "<<k
      <<", Number of inlier: "<<inls<<"("<<lastLn+lastPt<<")"<<endl;
  #endif

  //solve least squares
  int numInl;
  SetDataMatrix(dA, H, thr,thrAngle, numInl);

  if (numInl<5)
    return false;

  cvInitMatHeader(&A, numInl*2, 9, CV_64F, &dA);
  cvInitMatHeader(&Ut, numInl*2, 9, CV_64F, &dUt);

  ok = SolveHomography(&A, &W, &Ut, &Vt, H);

  if (ok && normalize)
  {
    DenormalizeH(H, T1, T2, H);
  }

  if (!ok || pow(1. - pow(eps,4),k) >= HOM_RANSAC_ETA0)
    return false;

  return true;
}





}

