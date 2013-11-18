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

void buildFromFiles(const std::vector<std::string> & ply_models, const std::vector<std::string> & sift_files, const std::vector<std::string> & model_names, std::vector<ObjectEntry> & out);

} // namespace blort_ros

#endif
