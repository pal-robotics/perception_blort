/*
 * @author:  Marko Mahnič
 * @created: 2010-05-11
 */
   
#ifndef XTGSERIALIZE_9NFCOXJ1
#define XTGSERIALIZE_9NFCOXJ1

#include <blort/TomGine/tgRenderModel.h>

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/list.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/string.hpp>


namespace boost {
namespace serialization {

template<class Archive>
void serialize(Archive & ar, TomGine::vec2 & v, const unsigned int version)
{
   ar & v.v;
}

template<class Archive>
void serialize(Archive & ar, TomGine::vec3 & v, const unsigned int version)
{
   ar & v.v;
}

template<class Archive>
void serialize(Archive & ar, TomGine::vec4 & v, const unsigned int version)
{
   ar & v.v;
}

template<class Archive>
void serialize(Archive & ar, TomGine::tgQuaternion & v, const unsigned int version)
{
   ar & v.x;
   ar & v.y;
   ar & v.z;
   ar & v.w;
}

template<class Archive>
void serialize(Archive & ar, TomGine::tgModel::Vertex & m, const unsigned int version)
{
   ar & m.pos;
   ar & m.normal;
   ar & m.texCoord;
}

template<class Archive>
void serialize(Archive & ar, TomGine::tgModel::Face & m, const unsigned int version)
{
   ar & m.vertices;
   ar & m.normal;
}

template<class Archive>
void serialize(Archive & ar, TomGine::tgModel::Line & m, const unsigned int version)
{
   ar & m.start;
   ar & m.end;
}

template<class Archive>
void serialize(Archive & ar, TomGine::tgModel::LineLoop & m, const unsigned int version)
{
   ar & m.points;
}

template<class Archive>
void serialize(Archive & ar, TomGine::tgModel & m, const unsigned int version)
{
   ar & m.m_vertices;
   ar & m.m_faces;
   ar & m.m_trianglefans;
   ar & m.m_quadstrips;
   ar & m.m_lines;
   ar & m.m_lineloops;
   ar & m.m_points;
}

template<class Archive>
void serialize(Archive & ar, TomGine::tgRenderModel::Material & m, const unsigned int version)
{
   ar & m.ambient;
   ar & m.diffuse;
   ar & m.specular;
   ar & m.color;
   ar & m.shininess;
}

template<class Archive>
void serialize(Archive & ar, TomGine::tgPose & m, const unsigned int version)
{
   ar & m.pos;
   ar & m.q;
}

template<class Archive>
void serialize(Archive & ar, TomGine::tgRenderModel & m, const unsigned int version)
{
    ar & boost::serialization::base_object<TomGine::tgModel>(m);
    ar & m.m_pose;
    ar & m.m_material;
}

} // namespace serialization
} // namespace boost

#endif /* end of include guard: XTGSERIALIZE_9NFCOXJ1 */
