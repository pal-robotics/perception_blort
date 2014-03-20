/**
 * @file TextureTracker.h
 * @author Thomas Mörwald
 * @date October 2009
 * @version 0.1
 * @brief Tracker using Texture for matching.
 * @namespace Tracker
 */
#ifndef _TEXTURE_TRACKER_H_
#define _TEXTURE_TRACKER_H_

#include <blort/Tracker/Tracker.h>
#include <opencv2/core/core.hpp>

namespace Tracking{

    /** @brief class TextureTracker */
    class TextureTracker : public Tracker
    {
    private:
	
	// Resources
	Shader* m_shadeTexEdgeTest;
	Shader* m_shadeTexColorTest;
	Shader* m_shadeConfidenceMM;
	Shader* m_shadeCompare;
	Texture* m_tex_model;
	std::vector<Texture*> m_tex_model_ip;
	Texture* m_tex_frame_cmp;
	Texture* m_tex_model_cmp;

	
	// Functions
	void model_processing(ModelEntry* modelEntry);
	
	void particle_filtering(ModelEntry* modelEntry);

    public:
	TextureTracker();
	~TextureTracker();

	virtual void setKernelSize(int val){
            params.kernel_size = val;
            m_shadeCompare->bind();
            m_shadeCompare->setUniform("kernelsize", params.kernel_size);
            m_shadeCompare->unbind();
	}
	
	virtual void setEdgeShader(){ 
            m_shadeCompare = m_shadeTexEdgeTest;
	}
	virtual void setColorShader(){
            m_shadeCompare = m_shadeTexColorTest;
	}
	
	virtual bool initInternal();
	
	virtual float evaluateParticle(ModelEntry* modelEntry);
	virtual float evaluateParticle(ModelEntry* modelEntry, Shader* shader);
	
	virtual void image_processing(unsigned char* image, GLenum format=GL_BGR);
	virtual void image_processing(unsigned char* image, const TomGine::tgModel &m, const TomGine::tgPose &p, GLenum format=GL_BGR);
	virtual void image_processing(unsigned char* image, int model_id, const TomGine::tgPose &p, GLenum format=GL_BGR);
	
	virtual bool track();
    bool track(const std::vector<bool> & tracking_objects);
	bool track(ModelEntry *modelEntry);
	virtual bool track(int id);
	
	virtual void textureFromImage(bool use_num_pixels=true);
	
	virtual void textureFromImage(int id, const TomGine::tgPose &pose, bool use_num_pixels=true);
	
	virtual void untextureModels();

	virtual void drawResult(float linewidth=2.0f);
	
	virtual void drawModelEntry(ModelEntry* modelEntry, float linewidth=1.0f);
	
	virtual void drawTrackerModel(int id, const TomGine::tgPose &p, float linewidth=1.0f);
	
	virtual void evaluatePDF( int id,
                                  float x_min, float y_min,
                                  float x_max, float y_max,
                                  int res,
                                  const char* meshfile, const char* xfile);

	
	virtual std::vector<float> getPDFxy(	ModelEntry* modelEntry,
                                                float x_min, float y_min,
                                                float x_max, float y_max,
                                                int res);

	virtual void savePDF(	std::vector<float> vPDFMap,
                                float x_min, float y_min,
                                float x_max, float y_max,
                                unsigned res,
                                const char* meshfile, const char* xfile);
	
        cv::Mat getModelTexture();

    };

} // namespace Tracking

#endif
