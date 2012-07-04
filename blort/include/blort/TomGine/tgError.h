
#ifndef _TG_ERROR_H
#define _TG_ERROR_H

#include <blort/TomGine/headers.h>
#include <string>

GLenum tgCheckError(std::string pre_msg);

GLenum tgCheckFBError(GLenum target, std::string pre_msg);

#endif
