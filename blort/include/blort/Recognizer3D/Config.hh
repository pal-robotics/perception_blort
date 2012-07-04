/**
 * $Id: Config.hh 34111 2012-07-03 14:29:54Z student5 $
 */

#ifndef P_CONFIG_HH
#define P_CONFIG_HH

#include <stdlib.h>
#include <string>
#include <map>
#include <blort/Recognizer3D/PNamespace.hh>

namespace P 
{

class Config
{
public:
  map<string, string> items;

public:
  Config() {}
  Config(const char *filename);
  void Load(const char *filename);
  void AddItem(const string &name, const string &value) {items[name] = value;}
  bool ExistItem(const string &name);
  string GetValueString(const string &name) {return items[name];}
  int GetValueInt(const string &name) {return atoi(items[name].c_str());}
  double GetValueDouble(const string &name) {return atof(items[name].c_str());}
};

}

#endif

