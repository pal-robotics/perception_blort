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

} // namespace blort_ros

#endif
