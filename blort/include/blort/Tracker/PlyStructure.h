 
#ifndef PLY_STRUCTURE
#define PLY_STRUCTURE

namespace Tracking{
  
struct PlyVertex {
	float x,y,z;                // spatial position
	float nx, ny, nz;           // normal vector
	float s, t;
	unsigned char r, g, b;      // color
};

struct PlyFace {
	unsigned short nverts;      // number of vertices used for the face (Quad=4, Triangle=3)
	unsigned int* v;            // pointer to memory holding the vertex-index list
	float t[3], b[3], n[3];			// Tangent space vectors
};

struct PlyEdge {
	unsigned short start;       // start vertex index
	unsigned short end;         // end vertex index
};

struct PlyPass {
	unsigned short nfaces;			// Number of faces using this pass
	unsigned int* f;						// pointer to memory holding the face-index list
	float m0,m1,m2,m3;					// matrix entries
	float m4,m5,m6,m7;					// matrix entries
	float m8,m9,m10,m11;					// matrix entries
	float m12,m13,m14,m15;					// matrix entries	
	float x,y,w,h;							// bounding box of texture with respect to modelview-projection-matrix
	unsigned short tex;					// index of texture
};

}

#endif