 /**
 * @file tgModel.h
 * @author Thomas Mörwald
 * @date October 2009
 * @version 0.1
 * @brief Defining a model for rendering.
 */

#ifndef TG_MODEL
#define TG_MODEL

#include <stdio.h>
#include <vector>

#include <blort/TomGine/tgMathlib.h>

namespace TomGine{

struct tgVertex{
	vec3 pos;				///< 3D position of vertex
	vec3 normal;			///< normal vector of vertex
	vec2 texCoord;			///< texture coordinate of vertex
};
struct tgFace{
	std::vector<int> v;		///< list of vertex-indices
	vec3 normal;			///< normal vector of face
};
struct tgLine{
	vec3 start;
	vec3 end;
};
struct tgRay{
	vec3 start;
	vec3 dir;
};
// struct LineLoop{
// 	std::vector<vec3> points;
// };

struct BoundingSphere{
	vec3 center;
	float radius;
};

/**
* @brief Class tgModel
*/
class tgModel{	
public:
	std::vector<tgVertex>	m_vertices;				///< list of vertices
	std::vector<tgFace>		m_faces;				///< list of faces
	std::vector<tgLine>		m_lines;				///< list of lines
	std::vector<vec3>		m_points;				///< list of points
	BoundingSphere  		m_bs;					///< bounding sphere
	
	// functions for access to member data
	void 		add(tgVertex v){ m_vertices.push_back(v); }
	void 		add(tgFace f){ m_faces.push_back(f); }
	tgVertex		getVertex(unsigned int i){ if(i<m_vertices.size()) return m_vertices[i]; else return tgVertex();}
	tgFace		getFace(unsigned int i){ if(i<m_faces.size()) return m_faces[i]; else return tgFace();}
	unsigned	getVertexSize(){ return m_vertices.size(); }
	unsigned	getFaceSize(){ return m_faces.size(); }
	
	/** @brief draws triangles and quadrangles given by m_faces */
	virtual void DrawFaces() const;
	
	/** @brief draws lines given by m_lines */
	virtual void DrawLines(const vec3 &color=vec3(1,0,0)) const;
	
	/** @brief draws points given by m_points */
	virtual void DrawPoints(const vec3 &color=vec3(1,0,0)) const;
	
	/** @brief draws normals of vertices in m_faces */
	virtual void DrawNormals(float normal_length) const;
	
	/** @brief Compute normals of vertices using cross product of faces */
	virtual void ComputeNormals();
	
	/** @brief computes normals of vertices of m_faces, m_polygons, m_quadstrips */
	virtual void ComputeFaceNormals();
	
	/** @brief computes bounding sphere which contains all vertices*/
	virtual void ComputeBoundingSphere();
	
	/** @brief clears data of model (m_vertices and m_faces) */
	virtual void Clear();
	
	/** @brief Prints infos of model to console */
	virtual void Print() const;
	
};

} // namespace TomGine

#endif
