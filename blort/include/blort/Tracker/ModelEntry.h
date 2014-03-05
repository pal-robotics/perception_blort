 /**
 * @file ModelEntry.h
 * @author Thomas Mörwald
 * @date January 2009
 * @version 0.1
 * @brief Main file of Tracker.
 * @namespace Tracking
 */
#ifndef _MODEL_ENTRY_H_
#define _MODEL_ENTRY_H_

#include <blort/TomGine/tgMathlib.h>
#include <blort/Tracker/TrackingStates.h>
#include <blort/Tracker/TrackerModel.h>
#include <blort/Tracker/Distribution.h>
#include <blort/Tracker/Predictor.h>
#include <blort/Tracker/Filter.h>

namespace Tracking{

/** @brief class ModelEntry */
class ModelEntry
{
private:
	void poseDiff(float &t, float &a);
	void speed();

	float max(float a, float b){ return (((a) > (b)) ? (a) : (b)); }
	float abs(float a){ return ((a>=0.0f) ? a : (-a)); }
	
	
	
public:
	ModelEntry();
	ModelEntry(const TomGine::tgModel& m);
	~ModelEntry();
	
	void setInitialPose(const TomGine::tgPose &p, float lpf_delay=0.0f, float lpf_delay_z=0.0f);
	void filter_pose();
	void evaluate_states(	const Particle &variation, unsigned rec,
							float c_th_base=0.6f, float c_th_min=0.3f, float c_th_fair=0.5f,
							float c_mv_not=0.01f, float c_mv_slow=0.05f, float c_th_lost=0.1f);
	
	std::string label;
	
	float t, a, t_max, a_max, c_edge, c_th, c_lost, abs_a, abs_t;
	
	movement_state st_movement;				///< movement state
	confidence_state st_confidence;			///< tracking confidence state
	quality_state st_quality;				///< qualitative state
	
	TrackerModel		model;				///< The model to track
	Distribution 		distribution;		///< Likelihood distribution
	Predictor*			predictor;			///< Movement prediction
	Texture*			mask;				///< Mask for model edges not to be considered (image space)
	Particle			pose;				///< Current pose of the model
	Particle			pose_prev;			///< Previouse pose of the model
	Particle			lpf_pose;		///< Low pass filtered pose;
	Particle			initial_pose;		///< Initial pose, pose where to reset model to
	
	float	speed_angular;					///< Current angular speed
	float	speed_translational;			///< Current translational speed

	float	confidence_color;				///< Confidence value of current pose with respect to color comparison
	float	confidence_edge;				///< Confidence value of current pose with respect to edge comparison
	
        int 	id;							///< ID of model (for removing)
	unsigned	num_convergence;			///< number of steps until convergence (static object);
	unsigned	hypothesis_id;				///< ID of model to compare with (if this ModelEntry is a hypothesis)
	std::vector<float> past_confidences;	///< vector of the past confidences
	unsigned	num_particles;				///< Number of particles used for representing likelihood distribution
	unsigned	num_recursions;				///< Number of recursions per image
	
	bool		bfc;						///< enable/disable backface culling
	bool		lock;						///< enable/disable lock
	bool		mask_geometry_edges;		///< enable/disable masking of geometry edges (use texture only)
	
	mat4 modelviewprojection;				///< Tranformation matrix from model to image coordinates
	TomGine::tgVector3 vCam2Model;			///< Vector from camera to model (for zooming DOF and texturing)
	
private:
	Predictor* del_predictor;
	SmoothFilter m_lpf_pose_tx;
	SmoothFilter m_lpf_pose_ty;
	SmoothFilter m_lpf_pose_tz;
	SmoothFilter m_lpf_pose_qx;
	SmoothFilter m_lpf_pose_qy;
	SmoothFilter m_lpf_pose_qz;
	SmoothFilter m_lpf_pose_qw;

public:
	SmoothFilter m_lpf_a;
	SmoothFilter m_lpf_t;
	SmoothFilter m_lpf_cs;
	SmoothFilter m_lpf_cl;

};

} // namespace Tracking

#endif
