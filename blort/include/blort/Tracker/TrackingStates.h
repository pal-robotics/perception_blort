#ifndef _H_TRACKINGSTATES_H_
#define _H_TRACKINGSTATES_H_

#include <ostream>

namespace Tracking{

enum quality_state{
    ST_OK,
    ST_OCCLUDED,
    ST_LOST,
    ST_LOCKED,
};

enum confidence_state{
    ST_GOOD,
    ST_FAIR,
    ST_BAD,
};

enum movement_state{
    ST_FAST,
    ST_SLOW,
    ST_STILL,
};

/* ostream operators for states enum */
inline std::ostream & operator<<(std::ostream & out, const Tracking::quality_state & st)
{
    switch(st)
    {
        case Tracking::ST_OK:
            out << "OK"; break;
        case Tracking::ST_OCCLUDED:
            out << "OCCLUDED"; break;
        case Tracking::ST_LOST:
            out << "LOST"; break;
        case Tracking::ST_LOCKED:
            out << "LOCKED"; break;
        default:
            out << "UNIMPLEMENTED"; break;
    }
    return out;
}

inline std::ostream & operator<<(std::ostream & out, const Tracking::confidence_state & st)
{
    switch(st)
    {
        case Tracking::ST_GOOD:
            out << "GOOD"; break;
        case Tracking::ST_FAIR:
            out << "FAIR"; break;
        case Tracking::ST_BAD:
            out << "BAD"; break;
        default:
            out << "UNIMPLEMENTED"; break;
    }
    return out;
}

inline std::ostream & operator<<(std::ostream & out, const Tracking::movement_state & st)
{
    switch(st)
    {
        case Tracking::ST_FAST:
            out << "FAST"; break;
        case Tracking::ST_SLOW:
            out << "SLOW"; break;
        case Tracking::ST_STILL:
            out << "STILL"; break;
        default:
            out << "UNIMPLEMENTED"; break;
    }
    return out;
}

}

#endif
