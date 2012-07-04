 /**
 * @file tgPose.h
 * @author Thomas Mörwald
 * @date October 2009
 * @version 0.1
 * @brief Defining the position and orientation of an object in 3D.
 */
 
#ifndef TG_POSE
#define TG_POSE

#include <blort/TomGine/tgMathlib.h>
#include <blort/TomGine/tgQuaternion.h>

namespace TomGine{

/**
* @brief Class tgPose
*/
class tgPose{
public:
	vec3 t;
	tgQuaternion q;
	
	tgPose operator*(const tgPose& p) const;
	vec3 operator*(const vec3& t) const;
	tgPose operator+(const tgPose &p) const;
	tgPose operator-(const tgPose &p) const;
	tgPose Transpose() const;
	
	void PrintHeader() const;
	void Print() const;

	void Activate() const;	
	void Deactivate() const;
	
	void SetPose(mat3 r, vec3 p);	
	void GetPose(mat3 &r, vec3 &p) const;
	
	void Rotate(float x, float y, float z);	
	void Rotate(vec3 r);
	void RotateAxis(vec3 rot);
	void RotateEuler(vec3 rot);
	void Translate(float x, float y, float z);	
	void Translate(vec3 t);	
};

} // namespace TomGine

#endif
