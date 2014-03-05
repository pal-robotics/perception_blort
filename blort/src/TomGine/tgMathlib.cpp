#include <blort/TomGine/tgMathlib.h>

std::ostream & operator<<(std::ostream & out, const mat3 & m)
{
    out << m.mat[0] << " " << m.mat[1] << " " << m.mat[2] << std::endl;
    out << m.mat[3] << " " << m.mat[4] << " " << m.mat[5] << std::endl;
    out << m.mat[6] << " " << m.mat[7] << " " << m.mat[8] << std::endl;
}
