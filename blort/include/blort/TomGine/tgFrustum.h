 /**
 * @file tgFrustum.h
 * @author Thomas Mörwald
 * @date October 2009
 * @version 0.1
 * @brief View frustum of a camera.
 */

#ifndef TG_FRUSTUM
#define TG_FRUSTUM

#include <blort/TomGine/headers.h>

#include <blort/TomGine/tgMathlib.h>

namespace TomGine{

/**
* @brief Class tgFrustum
*/
class tgFrustum{
    
private:
	float frustum[6][4];
	mat4 m_intrinsic;
	mat4 m_extrinsic;


public:
	void ExtractFrustum();
	bool PointInFrustum( float x, float y, float z );
	bool SphereInFrustum( float x, float y, float z, float radius );
	void DrawFrustum();

};

} // namespace TomGine

#endif
