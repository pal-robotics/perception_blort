/**
* @file tgShapeCreator.h
* @author Thomas Mörwald
* @date March 2010
* @version 0.1
* @brief Generating a sphere mesh using a subdevided icosahedra.
* @namespace TomGine
*/

#ifndef TGSHAPECREATOR_H
#define TGSHAPECREATOR_H

#include <vector>

#include <blort/TomGine/tgModel.h>

#define TETRAHEDRON 0
#define OCTAHEDRON 1
#define ICOSAHEDRON 2

namespace TomGine{

class tgShapeCreator{
private:
	int n_vertices;
	int n_faces;
	int n_edges;
	int edge_walk; 
	float *vertices;
	int *faces; 
	int *start; 
	int *end; 
	int *midpoint; 
	
 	void init_tetrahedron();
 	void init_octahedron();
 	void init_icosahedron();
 	
 	int search_midpoint(int index_start, int index_end);
	void subdivide();

public:
	tgShapeCreator();
 	
 	void CreateSphere(tgModel& model, float radius, int subdevisions, int method=0);
 	void CreateBox(tgModel& model, float x, float y, float z);
 	void CreateCylinder(tgModel &model, float radius, float height, int slices, int stacks, bool closed);
 	void CreateCone(tgModel &model, float radius, float height, int slices, int stacks, bool closed);
 	
 	void CreateConvexHull(tgModel &model, std::vector<vec3> points);
 	
 	void TriangulatePolygon(tgModel& model, std::vector<vec3> points);
 	
};

}
 
 #endif
