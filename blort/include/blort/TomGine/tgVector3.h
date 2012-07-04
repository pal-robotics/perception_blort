 /**
 * @file tgVector.h
 * @author Thomas Mörwald
 * @date October 2009
 * @version 0.1
 * @brief Defining a 3D vector. For comfortable camera movement.
 */
 
#ifndef TG_VECTOR3
#define TG_VECTOR3

#ifndef PI
#define PI 3.14159265358979323846f
#endif

#include <math.h>

namespace TomGine{

/**
* @brief Class tgVector3
*/
class tgVector3
{
public:
	float x, y, z;
	
	tgVector3();
	//tgVector3(const tgVector3 &v);
	tgVector3(float all);
	tgVector3(float x, float y, float z);

	tgVector3 operator=(tgVector3 v);
	tgVector3 operator+(tgVector3 v);
	tgVector3 operator-(tgVector3 v);
	tgVector3 operator*(tgVector3 v);
	tgVector3 operator*(float m);
	tgVector3 operator/(float m);
	
	tgVector3 cross(tgVector3 v);
	static tgVector3 cross(tgVector3 v1, tgVector3 v2);
	float dot(tgVector3 v);
	void normalize();
	float length();
	void setLength(float l);

	void rotateX(float fAngle);
	void rotateY(float fAngle);
	void rotateZ(float fAngle);

	void rotate(float fAngle, tgVector3 vAxis);

	//void GetRotYZ(float &fAngleY, float &fAngleZ);

};

// tgVector3 funktions
float Angle(tgVector3 a, tgVector3 b);

} // namespace TomGine

#endif
