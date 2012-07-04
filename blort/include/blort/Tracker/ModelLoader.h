 /**
 * @file ModelLoader.h
 * @author Thomas Mörwald
 * @date October 2009
 * @version 0.1
 * @brief Loading geometrical models from file
 * @namespace Tracking
 */
 
#ifndef _MODELLOADER_H_
#define _MODELLOADER_H_

#include <stdlib.h>
#include <stddef.h>
#include <string>

#include <blort/Tracker/ply.h>
#include <blort/Tracker/PlyStructure.h>
#include <blort/TomGine/tgModel.h>
#include <blort/Tracker/TrackerModel.h>


namespace Tracking{

/** @brief Loading geometrical models from file */
class ModelLoader
{
private:

	bool propertyIsInList(PlyProperty* prop, PlyProperty* list, int n, int* index);

public:
	ModelLoader();
	~ModelLoader();
	
	/** @brief Loads PLY (Polygon File Format, Stanford Triangle Format) 
	*		@param Model Storage for model
	*		@param filename path and filename of file to load
	*		@return true on success, false on failure
	*/
	bool LoadPly(TomGine::tgModel &model, const char* filename);
	bool LoadPly(TrackerModel &model, const char* filename);
	
	bool SavePly(TomGine::tgModel &model, const char* filename);
	bool SavePly(TrackerModel &model, const char* filename);
	   
};

} // namespace Tracking

#endif
