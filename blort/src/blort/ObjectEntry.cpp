#include <blort/ObjectEntry.h>

blort::RecogData blort::getBest(const ObjectEntry& obj)
{
  float best_conf = 0;
  size_t best_i = 0;
  for(size_t i = 0; i < obj.recog_data.size(); ++i)
  {
    if(obj.recog_data[i].conf > best_conf)
    {
      best_conf = obj.recog_data[i].conf;
      best_i = i;
    }
  }
  return obj.recog_data[best_i];
}

void blort::buildFromFiles(const std::vector<std::string> & ply_models, const std::vector<std::string> & sift_files, const std::vector<std::string> & model_names, std::vector<blort::ObjectEntry> & out)
{
  std::vector<size_t> sift_index(0);
  return buildFromFiles(ply_models, sift_files, model_names, out, sift_index);
}

void blort::buildFromFiles(const std::vector<std::string> & ply_models, const std::vector<std::string> & sift_files, const std::vector<std::string> & model_names, std::vector<blort::ObjectEntry> & out, std::vector<size_t> & sift_index)
{
  out.resize(0);
  sift_index.resize(0);
  for(size_t i = 0; i < model_names.size(); ++i)
  {
    blort::ObjectEntry entry;
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
        //entry.recog_data.push_back(sift_files[j]);
        const std::string s = sift_files[j];
        entry.recog_data.push_back(s.substr(s.find_last_of("\\/")+1));
      }
    }
    out.push_back(entry);
  }
}
