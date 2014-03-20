/**
 * Johann Prankl, 2010-01-27 
 * prankl@acin.tuwien.ac.at
 */

#ifndef P_SDRAW_HH
#define P_SDRAW_HH

/*#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>
*/
#include <opencv2/opencv.hpp>
#include <blort/Recognizer3D/PNamespace.hh>
#include <blort/Recognizer3D/Vector2.hh>
#include <blort/Recognizer3D/Array.hh>

namespace P
{

class SDraw
{
public:
  static void DrawCross(IplImage *img, double x, double y, 
                  CvScalar c=CV_RGB(0,0,255), int thickness=1);
  static void DrawLine(IplImage *img, double x1, double y1, double x2, double y2, 
                  CvScalar c=CV_RGB(255,255,255), int thickness=1);
  static void DrawCircle(IplImage *img, double x, double y, double r, 
                  CvScalar c=CV_RGB(255,255,255), int thickness=1);
  static void DrawEllipse(IplImage *img, double x, double y, double a, double b, 
                  double angle, CvScalar c=CV_RGB(255,255,255), int thickness=1);
  static void DrawArc(IplImage *img, double x, double y, double r, double start_angle,
                  double angular_span, CvScalar c=CV_RGB(255,0,0), int thickness=1);
  static void DrawTriangle(IplImage *img, double x1, double y1, double x2, double y2, 
                  double x3, double y3, CvScalar c=CV_RGB(0,0,255), int thickness=1);
  static void DrawRectangle(IplImage *img, double x1, double y1, double x2, double y2, 
                  CvScalar c=CV_RGB(0,0,255), int thickness=1);
  static void DrawPoly(IplImage *img, P::Array<Vector2> &vs,
                  CvScalar c=CV_RGB(0,0,255), int thickness=1);
  static void DrawFillPoly(IplImage *img, P::Array<Vector2> &vs,
                  CvScalar c=CV_RGB(0,0,255));
  static void DrawSIFT(IplImage *img, double x, double y, double l, double o, 
                  CvScalar c=CV_RGB(0,0,255), int thickness=1);
  static void WriteText(IplImage *img, const char* text, double x, double y, CvScalar c);
};


}

#endif

