 /**
 * @file tgCamera.h
 * @author Thomas Mörwald & Bence Magyar
 * @date April 2012
 * @version 0.2
 * @brief OpenGL Camera for moving around in 3D space (including internal and external camera parameters). \a
 *        Improved with a Parameter constructor which can handle a ROS camera_info message.
 */
 
#ifndef TG_CAMERA
#define TG_CAMERA

#include <blort/TomGine/headers.h>

#include <blort/TomGine/tgVector3.h>
#include <blort/TomGine/tgFrustum.h>
#include <blort/TomGine/tgMathlib.h>
#include <blort/TomGine/tgPose.h>
#include <sensor_msgs/CameraInfo.h>

#define GL_ORTHO 0
#define GL_PERSPECTIVE 1

namespace TomGine{

/**
* @brief Class tgCamera
*/
class tgCamera
{
private:
	// tgCamera definition
	tgVector3 m_vPos;			// Position of tgCamera (absolute)
	tgVector3 m_vView;		// Viewpoint of tgCamera (absolute)
	tgVector3 m_vUp;			// The tgCameras upside (relative)
	
	tgVector3 f;		// Vector of camera pointing forward 
	tgVector3 s;		// Vector of camera pointing sidewards (right)
	tgVector3 u;		// Vector of camera pointing up
	
	unsigned m_width, m_height;
	float m_fovy;
	float m_zNear, m_zFar;
	unsigned short m_projection;	
	mat4 m_extrinsic;
	mat4 m_intrinsic;
	
	tgFrustum m_frustum;
	
public:
	tgCamera();
	
	struct Parameter{
		Parameter();
                Parameter(const sensor_msgs::CameraInfo& cam_info);
                void setPose(const TomGine::tgPose& camPose);
		// image dimension
		unsigned width;
		unsigned height;
		// Instrinsic parameters:
		// entries of the camera matrix
		float fx;
		float fy;
		float cx;
		float cy;
		// radial distortion parameters
		float k1;
		float k2;
		float k3;
		// tangential distortion parameters
		float p1;
		float p2;
		// extrinsic parameters: 3D pose of camera w.r.t. world
		mat3 rot;
		vec3 pos;
		// Clipping planes of virtual camera
		float zNear;
		float zFar;
		
		void print(){
			printf("width = %u height = %u\n", width, height);
			printf("fx = %f fy = %f\n", fx, fy);
			printf("cx = %f cy = %f\n", cx, cy);
			printf("k1 = %f k2 = %f k3 = %f\n", k1, k2, k3);
			printf("p1 = %f p2 = %f\n", p1, p2);
			printf("rot\n %f %f %f\n", rot.mat[0], rot.mat[1], rot.mat[2]);
			printf(" %f %f %f\n", rot.mat[3], rot.mat[4], rot.mat[5]);
			printf(" %f %f %f\n", rot.mat[6], rot.mat[7], rot.mat[8]);
			printf("pos\n %f %f %f\n", pos.x, pos.y, pos.z);
			printf("zNear = %f zFar = %f\n", zNear, zFar);
		}
	};
	
	void Load(tgCamera::Parameter camPar);
	
	// Define tgCamera
	void Set(	float posx,  float posy,  float posz,
				float viewx, float viewy, float viewz,
				float upx,   float upy,   float upz,
				float fovy=45.0f, unsigned width=800, unsigned height=600,
				float zNear=0.1f, float zFar=100.0f,
				unsigned short projection=GL_PERSPECTIVE );
	void SetExtrinsic(float* M);
	void SetIntrinsic(float* M);
	void SetIntrinsic(float fovy, unsigned width, unsigned height, float zNear, float zFar, unsigned short projection);
	void SetViewport(unsigned w, unsigned h);
	void SetZRange(float near, float far);
	void SetPerspective(){m_projection=GL_PERSPECTIVE;}
	void SetOrtho(){m_projection=GL_ORTHO;}
	void SetPos(float x, float y, float z){ m_vPos.x=x; m_vPos.y=y; m_vPos.z=z; }
	
	vec2 ToImageSpace(const vec3 &world_space);
	
	void Activate();
	void Print();
	
	void pvu2fsu();
	void fsu2pvu();
	void fsu2extrinsic();
	void extrinsic2fsu();
	void fwh2intrinsic();
	
	// Gets
	TomGine::tgPose GetPose();

	tgVector3 GetF(){return f;}
	tgVector3 GetS(){return s;}
	tgVector3 GetU(){return u;}
	
	tgVector3 GetPos(){return m_vPos;}
	tgVector3 GetView(){return m_vView;}
	tgVector3 GetUp(){return m_vUp;}
	
	float GetZNear(){ return m_zNear; }
	float GetZFar(){ return m_zFar; }
	unsigned GetWidth(){ return m_width; }
	unsigned GetHeight(){return m_height; }
	
	float GetFOVY(){ return m_fovy; }
	unsigned short GetProjection(){ return m_projection; }
	mat4 GetIntrinsic(){ return m_intrinsic; }
	mat4 GetExtrinsic(){ return m_extrinsic; }
	
	tgFrustum* GetFrustum(){ return &m_frustum; }

	// Translations
	void Translate(tgVector3 v);
	void Translate(float x, float y, float z, float fWay);
	void TranslateF(float fWay);
	void TranslateS(float fWay);
	void TranslateU(float fWay);
	
	// Rotations
	void Rotate(float x, float y, float z, float fAngle);
	void RotateF(float fAngle);
	void RotateS(float fAngle);
	void RotateU(float fAngle);
	
	void RotateX(float fAngle);
	void RotateY(float fAngle);
	void RotateZ(float fAngle);
	
	void Orbit(tgVector3 vPoint, tgVector3 vAxis, float fAngle);
	
	// Movement
	void ApplyTransform();
	
	void DrawFrustum(){ m_frustum.DrawFrustum(); }

};

} // namespace TomGine

#endif
