#include "ObjectEntry.h"

void blort_ros::buildFromFiles(const std::vector<std::string> & ply_models, const std::vector<std::string> & sift_files, const std::vector<std::string> & model_names, std::vector<blort_ros::ObjectEntry> & out)
{
    std::vector<size_t> sift_index(0);
    return buildFromFiles(ply_models, sift_files, model_names, out, sift_index);
}

void blort_ros::buildFromFiles(const std::vector<std::string> & ply_models, const std::vector<std::string> & sift_files, const std::vector<std::string> & model_names, std::vector<blort_ros::ObjectEntry> & out, std::vector<size_t> & sift_index)
{
    out.resize(0);
    sift_index.resize(0);
    for(size_t i = 0; i < model_names.size(); ++i)
    {
        blort_ros::ObjectEntry entry;
        entry.name = model_names[i];
        for(size_t j = 0; j < ply_models.size(); ++j)
        {
            if(ply_models[j].find(entry.name) != std::string::npos)
            {
                entry.ply_model = ply_models[j];
                break;
            }
        }
        for(size_t j = 0; j < sift_files.size(); ++j)
        {
            if(sift_files[j].find(entry.name) != std::string::npos)
            {
                entry.sift_files.push_back(sift_files[j]);
            }
        }
        out.push_back(entry);
        if(i != 0)
        {
            sift_index.push_back(sift_index[i-1] + out[i-1].sift_files.size());
        }
        else
        {
            sift_index.push_back(0);
        }
    }
    sift_index.push_back(sift_index[sift_index.size()-1] + out[out.size()-1].sift_files.size());
}
