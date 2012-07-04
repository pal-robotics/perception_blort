/**
 * $Id: Object3D.cc 34111 2012-07-03 14:29:54Z student5 $
 * Johann Prankl, 2010-01-27 
 * prankl@acin.tuwien.ac.at
 */


#include <blort/Recognizer3D/Object3D.hh>

namespace P 
{

unsigned Object3D::idcnt=0;


/********************** Object3D ************************
 * Constructor/Destructor
 */
Object3D::Object3D()
 : id(UINT_MAX), conf(0.), err(DBL_MAX)
{
}

Object3D::~Object3D()
{
  DeleteCodebook(codebook);
}




/******************************** PUBLIC **************************/



void DeleteObjects3D(Array<Object3D*> &objects)
{
  for (unsigned i=0; i<objects.Size(); i++)
    delete objects[i];
  objects.Clear();
}



}

