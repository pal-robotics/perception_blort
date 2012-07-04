
#ifndef __TRACKER_MODEL_H__
#define __TRACKER_MODEL_H__

namespace Tracking{
	class TrackerModel;
}
#include <blort/Tracker/headers.h>
#include <blort/TomGine/tgModel.h>
#include <blort/Tracker/Texture.h>
#include <blort/Tracker/Shader.h>
#include <blort/TomGine/tgMathlib.h>
#include <blort/TomGine/tgPose.h>
#include <blort/TomGine/tgCamera.h>

#ifndef FN_LEN
#define FN_LEN 256
#endif

namespace Tracking{


/** @brief 3D Model with special methods for tracking and texturing */
class TrackerModel : public TomGine::tgModel
{
private:
	TrackerModel(const TrackerModel& m);
	
	TrackerModel& operator=(const TomGine::tgModel& m);		// no implementation (should not be used)

public:
	TrackerModel();
	TrackerModel(const TomGine::tgModel& m);
	~TrackerModel();
	
	TrackerModel& operator=(const TrackerModel& m);
	
	void releasePassList();
	
	struct Pass {												// Renderpass
		std::vector<unsigned> f;								// Faces to draw with this pass
		mat4 modelviewprojection;					// Modelview and projection matrix for texCoords
		float x,y,w,h;										// Bounding box of SubTexture
		Texture* texture;									// Texture to use
		Pass(){ texture = new(Texture); glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE); }
		~Pass(){ delete(texture);}
	};
	
	typedef std::vector<Pass*>	PassList;
	
	// Variables
	PassList				m_passlist;
	std::vector<int>		m_facepixellist;
	
	Texture* m_tex_original;		// original texture of model (not modified by tracker)
	Texture*	m_texture;				// texture of model modified by tracker (edge-texture)
	bool m_textured;
	
	// computes, updates
	void computeEdges();
	void computeBoundingSphere();
	void Update();
	
	// draws
	virtual void Print() const;
	virtual void drawNormals();
	virtual void drawFaces(bool colorful=false);
	void drawFace(int i);
	void drawEdges();
	void drawTexturedFaces();
	void drawUntexturedFaces();
	void drawPass(bool colorful=false);
	void drawCoordinates();
	
	
	std::vector<unsigned> getFaceUpdateList(TomGine::tgPose& p_max, vec3 view, float minTexGrabAngle=3.0*PI/4.0, bool use_num_pixels=true);
	
	void getBoundingBox2D( int width, int height, TomGine::tgPose& p_max, TomGine::tgCamera* m_cam,
							int &minX, int &maxX, int &minY, int &maxY );
	
	/** @brief capture texture from image */
	void textureFromImage(	Texture* image,
							int width, int height,
							TomGine::tgPose& p_max,
							vec3 view,
							float minTexGrabAngle,
							std::vector<unsigned> faceUpdateList,
							std::vector<TomGine::tgVertex> &vertices,
							TomGine::tgCamera* m_cam);
	
	void useTexCoords(bool useTC);
	void unwarpTexturesBox_hacky(const char* name);

		// gets
	bool 			getTextured(){ return m_textured; }
	Texture* 	getTexture(){ return m_texture; }
	Texture* 	getOriginalTexture(){ return m_tex_original; }
	float			getBoundingSphereRadius(){ return m_boundingSphereRadius; }
	
	// sets
	void setBFC(bool bfc){ m_bfc = bfc; }
	void setTexture(Texture* tex){ m_texture = tex; }
	void setOriginalTexture(Texture* tex){ m_tex_original = tex; }
	void restoreTexture(){ m_texture=m_tex_original; }
	
		// generate display lists
	void genListTexturedFaces();
	void genListUntexturedFaces();
	void genListPass(bool colorful=false);
	void genListFaces(bool colorful=false);
	void genListEdges();
	void genListNormals(float normal_length);
	
protected:
	GLint m_dlTexturedFaces;
	GLint m_dlUntexturedFaces;
	GLint m_dlPass;
	GLint m_dlFaces;
	GLint m_dlEdges;
	GLint m_dlNormals;
	
	Shader* m_shadeTexturing;
	int m_shadeTexturingID;
	bool m_bfc;
	float m_boundingSphereRadius;

	// Functions
	bool isRedundant(TomGine::tgLine* e1);
	void UpdateDisplayLists();
};

} // namespace Tracking

#endif
