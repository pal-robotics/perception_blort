 
#ifndef PLY_STRUCTURE
#define PLY_STRUCTURE

namespace TomGine{
  
struct PlyVertex {
	float x,y,z;                // spatial position
	float nx, ny, nz;           // normal vector
	float s, t;
	unsigned char r, g, b;      // color
};

struct PlyFace {
	unsigned short nverts;      // number of vertices used for the face (Quad=4, Triangle=3)
	unsigned int* v;            // pointer to memory holding the vertex-index list
};

struct PlyEdge {
	unsigned short start;       // start vertex index
	unsigned short end;         // end vertex index
};

}

#endif