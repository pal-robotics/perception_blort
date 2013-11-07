#include <blort/Tracker/ModelEntry.h>
#include <ros/console.h>

using namespace Tracking;

std::ostream & operator<<(std::ostream & out, const quality_state & st)
{
    switch(st)
    {
        case ST_OK:
            out << "OK"; break;
        case ST_OCCLUDED:
            out << "OCCLUDED"; break;
        case ST_LOST:
            out << "LOST"; break;
        case ST_LOCKED:
            out << "LOCKED"; break;
        default:
            out << "UNIMPLEMENTED"; break;
    }
    return out;
}

std::ostream & operator<<(std::ostream & out, const confidence_state & st)
{
    switch(st)
    {
        case ST_GOOD:
            out << "GOOD"; break;
        case ST_FAIR:
            out << "FAIR"; break;
        case ST_BAD:
            out << "BAD"; break;
        default:
            out << "UNIMPLEMENTED"; break;
    }
    return out;
}

std::ostream & operator<<(std::ostream & out, const movement_state & st)
{
    switch(st)
    {
        case ST_FAST:
            out << "FAST"; break;
        case ST_SLOW:
            out << "SLOW"; break;
        case ST_STILL:
            out << "STILL"; break;
        default:
            out << "UNIMPLEMENTED"; break;
    }
    return out;
}

/** @brief class ModelEntry */
ModelEntry::ModelEntry()
: m_lpf_a(0.7f), m_lpf_t(0.7f), m_lpf_cs(0.2f), m_lpf_cl(0.99f)
{  
	del_predictor = predictor = new Predictor();
	bfc = false;
	lock = false;
	mask_geometry_edges = false;
	num_convergence = 0;
  st_quality = ST_OK; //2012-11-28: added by Jordi
	mask = 0;
  ROS_WARN_STREAM("(" << label << ") ModelEntry(): st_quality = " << st_quality );
}

ModelEntry::ModelEntry(const TomGine::tgModel& m)
: model(m), m_lpf_a(0.7f), m_lpf_t(0.7f), m_lpf_cs(0.2f), m_lpf_cl(0.99f)
{
	del_predictor = predictor = new Predictor();
	bfc = false;
	lock = false;
	mask_geometry_edges = false;
	num_convergence = 0;
	mask = 0;
}
	
ModelEntry::~ModelEntry()
{
	delete(del_predictor);
}

void ModelEntry::poseDiff(float &t, float &a)
{
	vec3 axis0, axis1;
	float angle0, angle1;
	
	pose.q.getAxisAngle(axis0, angle0);
	pose_prev.q.getAxisAngle(axis1, angle1);
	
	vec3 axis01 = axis0 - axis1;
	vec3 t01 = pose.t - pose_prev.t;
	
	a = (angle0-angle1);
	t = t01.length();
}

void ModelEntry::speed()
{
	static Timer s_timer;
	float dt = (float)s_timer.Update();
	
	float t,a;
	poseDiff(t,a);
	
	speed_angular = a/dt;
	speed_translational = t/dt;
}

void ModelEntry::filter_pose()
{
	lpf_pose.t.x = m_lpf_pose_tx.Process(pose.t.x);
	lpf_pose.t.y = m_lpf_pose_ty.Process(pose.t.y);
	lpf_pose.t.z = m_lpf_pose_tz.Process(pose.t.z);
	lpf_pose.q.x = m_lpf_pose_qx.Process(pose.q.x);
	lpf_pose.q.y = m_lpf_pose_qy.Process(pose.q.y);
	lpf_pose.q.z = m_lpf_pose_qz.Process(pose.q.z);
	lpf_pose.q.w = m_lpf_pose_qw.Process(pose.q.w);
}

void ModelEntry::setInitialPose(const TomGine::tgPose &p, float lpf_delay, float lpf_delay_z)
{
	initial_pose = p;
	initial_pose.c = pose.c;
	initial_pose.w = pose.w;
	m_lpf_pose_tx.Set(p.t.x, lpf_delay);
	m_lpf_pose_ty.Set(p.t.y, lpf_delay);
	m_lpf_pose_tz.Set(p.t.z, lpf_delay_z);
	m_lpf_pose_qx.Set(p.q.x, lpf_delay);
	m_lpf_pose_qy.Set(p.q.y, lpf_delay);
	m_lpf_pose_qz.Set(p.q.z, lpf_delay);
	m_lpf_pose_qw.Set(p.q.w, lpf_delay);
}

void ModelEntry::evaluate_states(const Particle &variation, unsigned rec,
                                 float c_th_base, float c_th_min, float c_th_fair,
                                 float c_mv_not, float c_mv_slow, float c_th_lost)
{
	
  if(lock)
  {    
		st_quality = ST_LOCKED;
		return;
	}
	
	speed();
	
	poseDiff(t,a);
	
	t_max = max(variation.t.x, max(variation.t.y, variation.t.z));
	a_max = max(variation.r.x, max(variation.r.y, variation.r.z));
	
	t = t/(t_max * rec);
	a = a/(a_max * rec);

	t = m_lpf_t.Process(t);
	a = m_lpf_a.Process(a);
	
  c_edge = m_lpf_cs.Process(this->confidence_edge);  

	abs_a = abs(a);
	abs_t = abs(t);
	
	c_th = c_th_base -  max(abs_a, abs_t);
	if(c_th < c_th_min) c_th = c_th_min;
	
	if(c_edge < c_th)
		c_lost = m_lpf_cl.Process(c_th - c_edge);
	else
		c_lost = m_lpf_cl.Process(0.0f);  

	// Movement detection
	if( abs_a<c_mv_not && abs_t < c_mv_not )
		st_movement = ST_STILL;
	else if( abs_a<c_mv_slow && abs_t < c_mv_slow )
		st_movement = ST_SLOW;
	else
		st_movement = ST_FAST;

  ROS_INFO_STREAM("(" << label << ") ModeEntry: evaluate_states: c_edge = " << c_edge << "  c_th = " << c_th << " c_lost = " << c_lost << "  c_th_lost = " << c_th_lost);
  ROS_INFO_STREAM("                            st_movement = " << st_movement << "  st_quality = " << st_quality);
	
	//// Lost detection
  if( c_lost > c_th_lost )
  {
		if( st_movement != ST_STILL && st_quality != ST_OCCLUDED)
    {
			st_quality = ST_LOST;
    }
    else if( st_quality != ST_LOST)
    {
      st_quality = ST_OCCLUDED;
    }
  }
  else if( st_quality != ST_LOST && st_quality != ST_OCCLUDED )
  {
		st_quality = ST_OK;
	}

	if( st_quality == ST_OCCLUDED && st_movement == ST_FAST )
  {
		st_quality = ST_LOST;
  }   
	
	// Quality detection
	if( c_edge > c_th_base )
		st_confidence = ST_GOOD;
	else if( c_edge > c_th_fair )
		st_confidence = ST_FAIR;
	else
		st_confidence = ST_BAD;
		
  if( st_movement == ST_STILL && c_edge >= c_th_base )
  {
		st_quality = ST_OK;
	}

  ROS_INFO_STREAM("(" << label << ") ModelEntry::evaluate_states has set st_confidence = " << st_confidence << "  st_quality = " << st_quality);
	
}
