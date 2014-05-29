#ifndef _H_OBJECTENTRY_H_
#define _H_OBJECTENTRY_H_

#include <string>
#include <vector>

namespace blort_ros
{

struct ObjectEntry
{
    std::string name;
    std::string ply_model;
    std::vector<std::string> sift_files;
};

/* This build a vector of ObjectEntry from ply_models/sift_files/model_names vector */
void buildFromFiles(const std::vector<std::string> & ply_models, const std::vector<std::string> & sift_files, const std::vector<std::string> & model_names, std::vector<ObjectEntry> & out);

/* Additionnaly build a sift-index */
void buildFromFiles(const std::vector<std::string> & ply_models, const std::vector<std::string> & sift_files, const std::vector<std::string> & model_names, std::vector<ObjectEntry> & out, std::vector<size_t> & sift_index);

} // namespace blort_ros

#endif
