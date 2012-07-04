/**
 * $Id: CodebookEntry.cc 34111 2012-07-03 14:29:54Z student5 $
 * Johann Prankl, 2010-01-27 
 * prankl@acin.tuwien.ac.at
 */


#include <blort/Recognizer3D/CodebookEntry.hh>

namespace P 
{



CodebookEntry::CodebookEntry()
 : sqr_sigma(0),
   model(0),
   good(true),
   bad(true),
   cntGood(0),
   cntTime(0),
   reliability(0.)
{
}

CodebookEntry::CodebookEntry(KeypointDescriptor *k)
 : sqr_sigma(0),
   good(true),
   bad(true),
   cntGood(0),
   cntTime(0),
   reliability(0.)
{
  model = new KeypointDescriptor(k);
  occurrences.PushBack(new KeypointDescriptor(k));
}

CodebookEntry::~CodebookEntry()
{
  if (model!=0) delete model;

  Clear();
}




}

