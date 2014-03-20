/**
 */

#ifndef VECTOR3_HH
#define VECTOR3_HH

#include <iostream>
#include <blort/Recognizer3D/PNamespace.hh>
#include <blort/Recognizer3D/Except.hh>

namespace P 
{

/**
 *  Vector3 
 */
class Vector3 
{

public:
  double x, y, z;
  Vector3();
  Vector3(double xx, double yy, double zz);
  void Set(double xx, double yy, double zz){ x=xx; y=yy; z=zz;}
  Vector3& operator+=(const Vector3 &v);
  Vector3& operator-=(const Vector3 &v);
  Vector3& operator*=(double s);
  Vector3& operator/=(double s) throw(Except);
  double NormSquare() const;
  double LengthSquare() const;
  double Norm() const;
  double Length() const;
  void Normalise();

};

Vector3 operator-(const Vector3 &v);
bool operator==(const Vector3 &a, const Vector3 &b);
bool operator!=(const Vector3 &a, const Vector3 &b);
Vector3 operator+(const Vector3 &a, const Vector3 &b);
Vector3 operator-(const Vector3 &a, const Vector3 &b);
Vector3 operator*(const double s, const Vector3 &v);
Vector3 operator*(const Vector3 &v, const double s);
Vector3 operator/(const Vector3 &v, const double s) throw(Except);
double Length(const Vector3 &v);
Vector3 Normalise(const Vector3 &v);
double Dot(const Vector3 &a, const Vector3 &b);
Vector3 Cross(const Vector3 &a, const Vector3 &b);
Vector3 MidPoint(const Vector3 &a, const Vector3 &b);
double AngleBetween(const Vector3 &a, const Vector3 &b);
double DistanceSquare(const Vector3 &a, const Vector3 &b);
double Distance(const Vector3 &a, const Vector3 &b);
Vector3 PlaneExp2Normal(const Vector3 &a, const Vector3 &b, const Vector3 &c);

ostream& operator<<(ostream &os, const Vector3 &v);
istream& operator>>(istream &is, Vector3 &v) throw(Except);
string&       operator<<(string &s, const Vector3 &v);
const string& operator>>(const string &s, Vector3 &v);
}

#include <blort/Recognizer3D/Vector3.ic>

#endif
