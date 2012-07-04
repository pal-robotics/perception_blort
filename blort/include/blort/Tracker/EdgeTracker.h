 /**
 * @file EdgeTracker.h
 * @author Thomas Mörwald
 * @date October 2009
 * @version 0.1
 * @brief Tracker using geometry edges for matching.
 * @namespace Tracker
 */
#ifndef EDGE_TRACKER_H
#define EDGE_TRACKER_H

#include <blort/Tracker/Tracker.h>

namespace Tracking{

/** @brief class EdgeTracker */
class EdgeTracker : public Tracker
{
private:
	
	// Resources
	Shader* m_shadeEdgeCompare;

	// Functions
// 	void particle_processing(TrackerModel* model, Shader* shadeCompare);
	void particle_filtering(ModelEntry* modelEntry);
	
public:
	EdgeTracker();
	
	virtual bool initInternal();
	
	virtual void image_processing(unsigned char* image, GLenum format=GL_BGR);
	virtual void image_processing(unsigned char* image, const TomGine::tgModel &m, const TomGine::tgPose &p, GLenum format=GL_BGR);
	virtual void image_processing(unsigned char* image, int model_id, const TomGine::tgPose &p, GLenum format=GL_BGR){};
	
	virtual bool track();
	virtual bool track(int id);
						
	virtual void drawResult(float linewidth=1.0f);

};

} // namespace Tracking

#endif
