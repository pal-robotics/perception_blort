/**
 * $Id: ConfigFile.cc 34111 2012-07-03 14:29:54Z student5 $
 */

#include <blort/Recognizer3D/Except.hh>
#include <blort/Recognizer3D/ConfigFile.hh>

namespace P 
{

ConfigFile::ConfigFile(const string &name)
{
  line_cnt = 0;
  file.open(name.c_str(), ios::in);
  if(!file)
    throw Except(__HERE__, "failed to open file '%s'", name.c_str());
}

ConfigFile::~ConfigFile()
{
  file.close();
}

/**
* Reads the next line of a file into the given string ignoring comment lines.
* Returns 'true' if the line was filled. 'false' otherwise (i.e. if EOF was
* reached). The newline character is NOT appended.
* If the file was not opened readable then an exception is thrown.
*/
bool ConfigFile::GetLine(string &str)
{
  bool success = false;
  while(file && !success)
  {
    // get the next line
    str.erase();
    getline(file, str);
    line_cnt++;
    RemoveEOL(str);
    if(!IsComment(str))
      success = true;
  }
  return success;
}

/**
 * Remove end of line
 * Note that end of line may be `\n` (newline, unix style)
 * or '\r' (carriage return, dos style, wrong! very very wrong!!)
 * We have to check for both.
 */
void ConfigFile::RemoveEOL(string &str)
{
  string::size_type p = str.find_last_of("\n\r");
  if(p != string::npos)
    str.erase(p, 1);
}

/**
* Returns true if the line is a comment line.
* Comment lines are empty lines or start with "#".
*/
bool ConfigFile::IsComment(string &str)
{
  // skip leading whitespaces
  string::size_type p = str.find_first_not_of(" \f\n\r\t\v");
  if(p == string::npos)
    return true;
  if(str[p] == '#')
    return true;
  return false;
}

/**
 * Get the next word from a string, starting at pos.
 * Words are separated by whitespaces (as in isspace(3)). The string must
 * contain only a single line.
 * pos contains the position after the word or string::npos if at end of string.
 * Returns true if a word was found, false otherwise.
 */
bool GetWord(const string &str, string &word, string::size_type &pos)
{
  // skip leading whitespaces
  string::size_type s = str.find_first_not_of(" \f\n\r\t\v", pos);
  if(s != string::npos)
  {
    string::size_type e = str.find_first_of(" \f\n\r\t\v", s);
    if(e != string::npos)
      word = str.substr(s, e - s);
    else
      word = str.substr(s);
    pos = e;
    return true;
  }
  else
  {
    pos = s;
    return false;
  }
}

}

