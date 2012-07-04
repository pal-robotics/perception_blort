
#include <blort/TomGine/tgError.h>
#include <ros/console.h>

GLenum tgCheckError(std::string pre_msg)
{
    GLenum error = glGetError();
    switch(error){
    case GL_NO_ERROR:
        // No error has been recorded.
        // The value of this symbolic constant is guaranteed to be 0.
        break;
    case GL_INVALID_ENUM:
        // An unacceptable value is specified for an enumerated argument.
        // The offending command is ignored
        // and has no other side effect than to set the error flag.
        ROS_ERROR("%s OpenGL Error: GL_INVALID_ENUM\n", pre_msg.c_str());
        break;
    case GL_INVALID_VALUE:
        // A numeric argument is out of range.
        // The offending command is ignored
        // and has no other side effect than to set the error flag.
        ROS_ERROR("%s OpenGL Error: GL_INVALID_VALUE\n", pre_msg.c_str());
        break;
    case GL_INVALID_OPERATION:
        // The specified operation is not allowed in the current state.
        // The offending command is ignored
        // and has no other side effect than to set the error flag.
        ROS_ERROR("%s OpenGL Error: GL_INVALID_OPERATION\n", pre_msg.c_str());
        break;
    case GL_INVALID_FRAMEBUFFER_OPERATION:
        // The framebuffer object is not complete. The offending command
        // is ignored and has no other side effect than to set the error flag.
        ROS_ERROR("%s OpenGL Error: GL_INVALID_FRAMEBUFFER_OPERATION\n", pre_msg.c_str());
        break;
    case GL_OUT_OF_MEMORY:
        // There is not enough memory left to execute the command.
        // The state of the GL is undefined, except for the state of the error flags,
        // after this error is recorded.
        ROS_ERROR("%s OpenGL Error: GL_OUT_OF_MEMORY\n", pre_msg.c_str());
        break;
    default:
        ROS_ERROR("%s OpenGL Error: unknown\n", pre_msg.c_str());
        break;
    }
    return error;
}

GLenum tgCheckFBError(GLenum target, std::string pre_msg){

    GLenum error = glCheckFramebufferStatus(target);

    switch(error){
    case GL_FRAMEBUFFER_COMPLETE:
        break;
    case GL_FRAMEBUFFER_UNDEFINED:
        // is returned if target is the default framebuffer, but the default framebuffer does not exist.
        ROS_ERROR("%s OpenGL FBO Error: GL_FRAMEBUFFER_UNDEFINED\n", pre_msg.c_str());
        break;
    case GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT:
        // is returned if any of the framebuffer attachment points are framebuffer incomplete.
        ROS_ERROR("%s OpenGL FBO Error: GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT\n", pre_msg.c_str());
        break;
    case GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT:
        // is returned if the framebuffer does not have at least one image attached to it.
        ROS_ERROR("%s OpenGL FBO Error: GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT\n", pre_msg.c_str());
        break;
    case GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER:
        // is returned if the value of GL_FRAMEBUFFER_ATTACHMENT_OBJECT_TYPE
        // is GL_NONE for any color attachment point(s) named by GL_DRAWBUFFERi.
        ROS_ERROR("%s OpenGL FBO Error: GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER\n", pre_msg.c_str());
        break;
    case GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER:
        // is returned if the value of GL_FRAMEBUFFER_ATTACHMENT_OBJECT_TYPE
        // is GL_NONE for any color attachment point(s) named by GL_DRAWBUFFERi.
        ROS_ERROR("%s OpenGL FBO Error: GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER\n", pre_msg.c_str());
        break;
    case GL_FRAMEBUFFER_UNSUPPORTED:
        // is returned if the combination of internal formats of the attached images violates
        // an implementation-dependent set of restrictions.
        ROS_ERROR("%s OpenGL FBO Error: GL_FRAMEBUFFER_UNSUPPORTED\n", pre_msg.c_str());
        break;
    case GL_FRAMEBUFFER_INCOMPLETE_MULTISAMPLE:
        // is returned if the value of GL_RENDERBUFFER_SAMPLES is not the same
        // for all attached renderbuffers; if the value of GL_TEXTURE_SAMPLES is the not same for all attached textures;
        // or, if the attached images are a mix of renderbuffers and textures, the value of GL_RENDERBUFFER_SAMPLES
        // does not match the value of GL_TEXTURE_SAMPLES.
        ROS_ERROR("%s OpenGL FBO Error: GL_FRAMEBUFFER_INCOMPLETE_MULTISAMPLE\n", pre_msg.c_str());
        break;
    default:
        ROS_ERROR("%s OpenGL FBO Error: unknown\n", pre_msg.c_str());
        break;
    }
    return error;
}

