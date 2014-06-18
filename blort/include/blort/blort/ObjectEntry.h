#ifndef _H_OBJECTENTRY_H_
#define _H_OBJECTENTRY_H_

#include <string>
#include <vector>
#include <map>
#include <blort/TomGine/tgPose.h>
#include <blort/Tracker/TrackingStates.h>

namespace blort
{

struct RecogData
{
  RecogData(){}
  RecogData(std::string sift_fn): sift_file(sift_fn) {}
  std::string sift_file;
  double conf;
  TomGine::tgPose pose;
};

struct ObjectEntry
{
    std::string name;
    std::string ply_model;
    std::vector<RecogData> recog_data;

    // message fields of blort_msgs::TrackerConfidence
    double edgeConf;
    double confThreshold;
    double lostConf;
    double distance;

    // Tracker data
    Tracking::movement_state movement;
    Tracking::quality_state quality;
    Tracking::confidence_state tracking_conf;
};

/*
 *@return the name of the sift file with the best confidence
 */
RecogData getBest(const ObjectEntry& obj);

/* This build a vector of ObjectEntry from ply_models/sift_files/model_names vector */
void buildFromFiles(const std::vector<std::string> & ply_models, const std::vector<std::string> & sift_files, const std::vector<std::string> & model_names, std::vector<ObjectEntry> & out);

/* Additionnaly build a sift-index */
void buildFromFiles(const std::vector<std::string> & ply_models, const std::vector<std::string> & sift_files, const std::vector<std::string> & model_names, std::vector<ObjectEntry> & out, std::vector<size_t> & sift_index);

} // namespace

#endif
