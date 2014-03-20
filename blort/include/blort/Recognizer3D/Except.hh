/**
 */

#ifndef P_EXCEPT_HH
#define P_EXCEPT_HH

#include <stdexcept>
#include <blort/Recognizer3D/PNamespace.hh>

#define __HERE__   __FILE__, __FUNCTION__, __LINE__

namespace P
{

/**
 * An informative exception class.
 * Example:
 *   Except(__FILE__, __FUNCTION__, __LINE__, "There were %d %s in the tree.",
 *          42, "elephants");
 * output:
 *   "DumbFile.cc:StupidFunction:28: There were 42 elephants in the tree."
 * Note: You can use the __HERE__ macro to get shorter statements:
 *   Except(__HERE__, "There were %d %s in the tree.", 42, "elephants");
 */
class Except : public exception
{
  string _what;
public:
  Except(const char *file, const char *function, int line,
         const char *format, ...) throw();
  virtual ~Except() throw() {}
  virtual const char* what() const throw() {return _what.c_str();}
  void Set(const string &s) {_what = s;}
};

}

#endif

