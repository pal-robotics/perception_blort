/**
 * $Id: Geometry.cc 34111 2012-07-03 14:29:54Z student5 $
 * Johann Prankl, 2010-01-27 
 * prankl@acin.tuwien.ac.at
 * TODO:
 * - Bug in ChainHull2D
 */


#include <blort/Recognizer3D/Geometry.hh>


namespace P 
{

#define NONE  (-1)

//for convex hull
typedef struct range_bin Bin;
 struct range_bin {
     int    min;    // index of min point P[] in bin (>=0 or NONE)
     int    max;    // index of max point P[] in bin (>=0 or NONE)
 };



/**
 * Calculate hullpoints
 *
 * Copyright 2001, softSurfer (www.softsurfer.com)
 * This code may be freely used and modified for any purpose
 * providing that this copyright notice is included with it.
 * SoftSurfer makes no warranty for this code, and cannot be held
 * liable for any real or imagined damage resulting from its use.
 * Users of this code must verify correctness for their application.
 *
 * nearHull_2D(): the BFP fast approximate 2D convex hull algorithm
 *     Input:  p[] = an (unsorted) array of 2D points
 *     Output: h[] = an array of the convex hull vertices (max is n)
 */
void ChainHull2D( Array<Vector2> &p, Array<Vector2> &h)
{
  h.Clear();               //delete hull
  if (p.Size()>0){
    int    minmin=0,  minmax=0;
    int    maxmin=0,  maxmax=0;
    int k = 200;                 //CHULL_MAX;
    Vector2* cP;                 // the current point being considered
    int    bot=0, top=(-1);  // indices for bottom and top of the stack

    int n=p.Size();
    float  xmin=p[0].x,  xmax=p[0].x;

    // Get the points with (1) min-max x-coord, and (2) min-max y-coord
    for (int i=1; i<n; i++) {
        cP = &p[i];
        if (cP->x <= xmin) {
            if (cP->x < xmin) {        // new xmin
                xmin = cP->x;
                minmin = minmax = i;
            }
            else {                      // another xmin
                if (cP->y < p[minmin].y)
                    minmin = i;
                else if (cP->y > p[minmax].y)
                    minmax = i;
            }
        }

        if (cP->x >= xmax) {
             if (cP->x > xmax) {        // new xmax
                 xmax = cP->x;
                 maxmin = maxmax = i;
             }
             else {                      // another xmax
                 if (cP->y < p[maxmin].y)
                     maxmin = i;
                 else if (cP->y > p[maxmax].y)
                     maxmax = i;
             }
        }
    }

    if (xmin == xmax) {      // degenerate case: all x-coords == xmin
         h.PushBack(p[minmin]);             // a point, or
         top++;
         if (minmax != minmin){           // a nontrivial segment
             top++;
             h.PushBack(p[minmax]);
         }

    }else{
       // Next, get the max and min points in the k range bins
       Bin*   B = new Bin[k+2];   // first allocate the bins
       B[0].min = minmin;         B[0].max = minmax;        // set bin 0
       B[k+1].min = maxmin;       B[k+1].max = maxmax;      // set bin k+1
       for (int b=1; b<=k; b++) { // initially nothing is in the other bins
           B[b].min = B[b].max = NONE;
       }

       for (int b, i=0; i<n; i++) {
          cP = &p[i];
          if (cP->x == xmin || cP->x == xmax) // already have bins 0 and k+1
              continue;

          // check if a lower or upper point
          if (IsLeft( p[minmin], p[maxmin], *cP) < 0) {  // below lower line
              b = (int)( k * (cP->x - xmin) / (xmax - xmin) ) + 1;  // bin #

              if (B[b].min == NONE){       // no min point in this range
                  B[b].min = i;           // first min
              }else if (cP->y < p[B[b].min].y){
                  B[b].min = i;           // new min
              }
              continue;
          }

          if (IsLeft( p[minmax], p[maxmax], *cP) > 0) {  // above upper line
              b = (int)( k * (cP->x - xmin) / (xmax - xmin) ) + 1;  // bin #

              if (B[b].max == NONE){       // no max point in this range
                  B[b].max = i;           // first max
              }else if (cP->y > p[B[b].max].y){
                  B[b].max = i;           // new max
              }
              continue;
          }
       }

       // Now, use the chain algorithm to get the lower and upper hulls
       // the output array H[] will be used as the stack
       // First, compute the lower hull on the stack H
       int idx;
       for (int i=0; i <= k+1; ++i)
       {
         if (B[i].min == NONE)  // no min point in this range
               continue;
           cP = &p[ B[i].min ];   // select the current min point
           idx=B[i].min;

           while (top > 0)        // there are at least 2 points on the stack
           {
               // test if current point is left of the line at the stack top
               if (IsLeft( h[top-1], h[top], *cP) > 0)
                  break;         // cP is a new hull vertex
               else{
                   top--;         // pop top point off stack
                   h.EraseLast();
               }
           }
           top++;

           h.PushBack(p[idx]);        // push current point onto stack
       }

       // Next, compute the upper hull on the stack H above the bottom hull
       if (maxmax != maxmin){      // if distinct xmax points
           top++;

           h.PushBack(p[maxmax]);   // push maxmax point onto stack
       }

       bot = top;                 // the bottom point of the upper hull stack
       for (int i=k; i >= 0; --i)
       {
         if (B[i].max == NONE)  // no max point in this range
               continue;
           cP = &p[ B[i].max ];   // select the current max point
           idx=B[i].max;

           while (top > bot)      // at least 2 points on the upper stack
           {
               // test if current point is left of the line at the stack top
               if (IsLeft( h[top-1], h[top], *cP) > 0)
                   break;         // current point is a new hull vertex
               else{
                   top--;         // pop top point off stack
                   h.EraseLast();
               }
           }
           top++;
           h.PushBack(p[idx]);      // push current point onto stack
       }

       if (minmax != minmin){
           top++;
           h.PushBack(p[minmin]);   // push joining endpoint onto stack
       }
       delete[] B;                  // free bins before returning
    }

    //ups a bug
    if (h.Size()>0)
      h.Erase(h.Size()-1);      //now it is fixed ;-)
  }
}

void ConvexHull( Array<Vector2> &p, Array<Vector2> &h)
{
  h.Clear();
  CvPoint* points = (CvPoint*)malloc( p.Size() * sizeof(points[0]));
  int* hull = (int*)malloc( p.Size() * sizeof(hull[0]));

  for( unsigned i = 0; i < p.Size(); i++ )
  {
    points[i].x = (int)p[i].x;
    points[i].y = (int)p[i].y;
  }

  CvMat hullMat = cvMat( 1, p.Size(), CV_32SC1, hull );
  CvMat pointMat = cvMat( 1, p.Size(), CV_32SC2, points);

  cvConvexHull2( &pointMat, &hullMat, CV_CLOCKWISE, 0 );
  int hullcount = hullMat.cols;

  for (int i=0; i<hullcount; i++)
    h.PushBack(p[hull[i]]);

  free( hull );
  free( points);
}

/**
 * AngleHalf
 * finds half an angle.
 * The original angle is defined by the sequence of points P1, P2 and P3.
 *        P1
 *        /
 *       /   P4
 *      /  .
 *     / .
 *    P2--------->P3
 */
void AngleHalf(const Vector2 &p1, const Vector2 &p2, const Vector2 &p3, 
               Vector2 &p4)
{
  double norm;

  norm = sqrt ( ( p1.x - p2.x ) * ( p1.x - p2.x )
              + ( p1.y - p2.y ) * ( p1.y - p2.y ) );


  p4.x = ( p1.x - p2.x ) / norm;
  p4.y = ( p1.y - p2.y ) / norm;

  norm = sqrt ( ( p3.x - p2.x ) * ( p3.x - p2.x )
              + ( p3.y - p2.y ) * ( p3.y - p2.y ) );

  p4.x = p4.x + ( p3.x - p2.x ) / norm;
  p4.y = p4.y + ( p3.y - p2.y ) / norm;

  p4.x = 0.5 * p4.x;
  p4.y = 0.5 * p4.y;

  norm = sqrt ( p4.x * p4.x + p4.y * p4.y );

  p4.x = p2.x + p4.x / norm;
  p4.y = p2.y + p4.y / norm;
}

/**
 * AngleRAD
 * returns the angle in radians swept out between two rays
 * AngleRAD ( P1, P2, P3 ) + AngleRAD ( P3, P2, P1 ) = 2 * PI
 *
 *        P1
 *        /
 *       /
 *      /
 *     /
 *    P2--------->P3
 */
double AngleRAD(const Vector2 &p1, const Vector2 &p2, const Vector2 &p3)
{
#define _PI 3.141592653589793

  Vector2 p;
  double value;

  p.x = ( p3.x - p2.x ) * ( p1.x - p2.x )
       + ( p3.y - p2.y ) * ( p1.y - p2.y );


  p.y = ( p3.x - p2.x ) * ( p1.y - p2.y )
       - ( p3.y - p2.y ) * ( p1.x - p2.x );

  if ( p.x == 0.0 && p.y == 0.0 )
  {
    value = 0.0;
    return value;
  }

  value = atan2 ( p.y, p.x );

  if ( value < 0.0 )
  {
    value = value + 2.0 * _PI;
  }

  return value;
# undef _PI
}

}

