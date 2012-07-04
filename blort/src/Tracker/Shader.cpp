/* Shader
 *
 * Copyright (C) 2005, Maurizio Monge <maurizio.monge@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <blort/Tracker/Shader.h>
#include <blort/Tracker/ShaderUtils.h>
#include <string.h>
#include <stdarg.h>
#include <iostream>
#include <ros/console.h>

#ifdef HAVE_GTK
#include <gtk/gtk.h>
#endif

#include <GL/glu.h>

using namespace Tracking;

static const char *gl_type_name(GLenum type)
{
    switch(type)
    {
        case GL_FLOAT:             return "float";
        case GL_FLOAT_VEC2_ARB:    return "vec2";
        case GL_FLOAT_VEC3_ARB:    return "vec3";
        case GL_FLOAT_VEC4_ARB:    return "vec4";
        case GL_FLOAT_MAT2_ARB:    return "mat2";
        case GL_FLOAT_MAT3_ARB:    return "mat3";
        case GL_FLOAT_MAT4_ARB:    return "mat4";
        case GL_INT:               return "int";
        case GL_INT_VEC2_ARB:      return "ivec2";
        case GL_INT_VEC3_ARB:      return "ivec3";
        case GL_INT_VEC4_ARB:      return "ivec4";
        case GL_BOOL_ARB:          return "bool";
        case GL_BOOL_VEC2_ARB:     return "bvec2";
        case GL_BOOL_VEC3_ARB:     return "bvec3";
        case GL_BOOL_VEC4_ARB:     return "bvec4";
        case GL_SAMPLER_1D:        return "sampler1D";
        case GL_SAMPLER_2D:        return "sampler2D";
        case GL_SAMPLER_3D:        return "sampler3D";
        case GL_SAMPLER_CUBE:      return "samplerCube";
        case GL_SAMPLER_1D_SHADOW: return "sampler1DShadow";
        case GL_SAMPLER_2D_SHADOW: return "sampler2DShadow";
        default:
        {
            static char tmp[64];
            snprintf(tmp,64,"?0x%x?", type );
            return tmp;
        }
    }
}

void Shader::dumpVars()
{
    int nv;
    glGetObjectParameterivARB( program, GL_OBJECT_ACTIVE_UNIFORMS_ARB, &nv);
    ROS_DEBUG("UNIFORM variables (%d)",nv);

    for(int i=0;i<nv;i++)
    {
        GLenum type;
        int size;
        char vname[4096];
        glGetActiveUniformARB(program,i,4096,NULL,&size,&type,vname);
        ROS_DEBUG("  %s %s;\n", gl_type_name(type),vname);
    }

    glGetObjectParameterivARB( program, GL_OBJECT_ACTIVE_ATTRIBUTES_ARB, &nv);
    ROS_DEBUG("ATTRIBUTE variables (%d):\n",nv);
    for(int i=0;i<nv;i++)
    {
        GLenum type;
        int size;
        char vname[4096];
        glGetActiveAttribARB(program,i,4096,NULL,&size,&type,vname);
        ROS_DEBUG("  %s %s;\n", gl_type_name(type),vname);
    }
}

void Shader::printInfoLog(GLhandleARB obj, const char *msg, ...)
{
    int infologLength = 0;
    int charsWritten  = 0;
    char *infoLog;

    glGetObjectParameterivARB(obj, GL_OBJECT_INFO_LOG_LENGTH_ARB,
                              &infologLength);
    if(infologLength > 1)
    {
        va_list va;
        va_start(va, msg);
        infoLog = (char *)malloc(infologLength);
        glGetInfoLogARB(obj, infologLength, &charsWritten, infoLog);
#ifdef HAVE_GTK
        char *m = g_strdup_vprintf(msg, va);
        GtkWidget *dialog = gtk_message_dialog_new(NULL,GTK_DIALOG_MODAL,
                GTK_MESSAGE_ERROR,GTK_BUTTONS_OK,"%s (%d):",m,infologLength);
        g_free(m);
        gtk_message_dialog_format_secondary_text(GTK_MESSAGE_DIALOG(dialog),"%s",infoLog);
        gtk_dialog_run(GTK_DIALOG(dialog));
        gtk_widget_destroy(dialog);
#else
        vprintf(msg, va);
        ROS_DEBUG(" (%d):\n",infologLength);
        ROS_DEBUG("%s",infoLog);
#endif
        va_end(va);
        free(infoLog);
    }
}

bool Shader::getStatus(){
    if(program)
        return true;
    
    std::cerr << "No program" << std::endl;
    return false;
}

Shader::Shader(const char *vertex_file, const char *fragment_file, const char *header)
{
    GLint status;
    const char *vs = read_text_file(vertex_file);
    const char *fs = read_text_file(fragment_file);
    
	if(header)  {
        if(vs) {
            char *tmp = (char*)malloc(strlen(header)+strlen(vs)+1);
            strcpy(tmp,header);
            strcat(tmp,vs);
            free((void*)vs);
            vs = tmp;
        }
        if(fs) {
            char *tmp = (char*)malloc(strlen(header)+strlen(fs)+1);
            strcpy(tmp,header);
            strcat(tmp,fs);
            free((void*)fs);
            fs = tmp;
        }
    }

    if(vs)
    {
        vertex = glCreateShaderObjectARB(GL_VERTEX_SHADER_ARB);
        glShaderSourceARB(vertex, 1, &vs, NULL);
        glCompileShaderARB(vertex);

        glGetObjectParameterivARB( vertex, GL_OBJECT_COMPILE_STATUS_ARB, &status);
        if(!status)
        {
            std::cerr << "compile vert" << std::endl;
            printInfoLog(vertex,"[Shader::Shader] Error compiling vertex shader \"%s\"",vertex_file);
            program = 0;
            return;
        }
        free((void*)vs);
    }
    else
        vertex = 0;

    if(fs)
    {
        fragment = glCreateShaderObjectARB(GL_FRAGMENT_SHADER_ARB);
        glShaderSourceARB(fragment, 1, &fs,NULL);
        glCompileShaderARB(fragment);

        glGetObjectParameterivARB( fragment, GL_OBJECT_COMPILE_STATUS_ARB, &status);
        if(!status)
        {
            std::cerr << "compile frag" << std::endl;
            printInfoLog(fragment,"[Shader::Shader] Error compiling fragment shader \"%s\"",fragment_file);
            program = 0;
            return;
        }
        free((void*)fs);
    }
    else
        fragment = 0;

    if(fragment==0 && vertex==0)
    {
        std::cerr << "fragment==0 and vertex==0" << std::endl;
        program = 0;
        return;
    }

    program = glCreateProgramObjectARB();
    if(vertex!=0)
    {
        glAttachObjectARB(program,vertex);
        glDeleteObjectARB(vertex);
    }
    if(fragment!=0)
    {
        glAttachObjectARB(program,fragment);
        glDeleteObjectARB(fragment);
    }
    glLinkProgramARB(program);

    glGetObjectParameterivARB( program, GL_OBJECT_LINK_STATUS_ARB, &status);
    if(!status)
    {
        std::cerr << "linking" << std::endl;
        printInfoLog(program,"Error linking program with \"%s\" and \"%s\"",
                                                    vertex_file,fragment_file);
        glDeleteObjectARB(program);
        program = 0;
        return;
    }

    glValidateProgramARB(program);
    glGetObjectParameterivARB( program, GL_OBJECT_VALIDATE_STATUS_ARB,&status);
    printInfoLog(program,"%s validating program",status?"Information":"Error");
    if(!status)
    {
        std::cerr << "validation" << std::endl;
        glDeleteObjectARB(program);
        program = 0;
    }

    //dumpVars();
}

Shader::~Shader()
{
    if(program!=0){
        glDeleteObjectARB(program);
    }
}

void Shader::bind()
{
    if(program)
        glUseProgramObjectARB(program);
}

void Shader::unbind()
{
    if(program)
        glUseProgramObjectARB(0);
}

GLuint Shader::getAttribLoc(const char *attr)
{
    return glGetAttribLocationARB( program, attr);
}

GLint Shader::getUniformLoc(const char* var)
{
    return glGetUniformLocationARB(program,var);
}

void Shader::setUniform(const char* var,int f)
{
    int loc = glGetUniformLocationARB(program,var);
    glUniform1iARB(loc,f);
}

void Shader::setUniform(const char* var,unsigned f)
{
    int loc = glGetUniformLocationARB(program,var);
    glUniform1iARB(loc,(int)f);
}

void Shader::setUniform(const char* var,int n,const int* f)
{
    int loc = glGetUniformLocationARB(program,var);
    glUniform1ivARB(loc,n,f);
}

void Shader::setUniform(const char* var,float f)
{
    int loc = glGetUniformLocationARB(program,var);
    glUniform1fARB(loc,f);
}

void Shader::setUniform(const char* var,int n,const float* f)
{
    int loc = glGetUniformLocationARB(program,var);
    glUniform1fvARB(loc,n,f);
    //ROS_DEBUG("f: %f %f %f\n", f[0],f[1],f[2]);
}

void Shader::setUniform(const char* var,vec2 f)
{
    int loc = glGetUniformLocationARB(program,var);
    glUniform2fvARB(loc,1,f.v);
}

void Shader::setUniform(const char* var,int n,vec2* f)
{
    int loc = glGetUniformLocationARB(program,var);
    glUniform1fvARB(loc,2*n,f->v);
}

void Shader::setUniform(const char* var,vec3 f)
{
    int loc = glGetUniformLocationARB(program,var);
    glUniform3fvARB(loc,1,f.v);
}

void Shader::setUniform(const char* var,int n,vec3* f)
{
    int loc = glGetUniformLocationARB(program,var);
    glUniform1fvARB(loc,3*n,f->v);
}

void Shader::setUniform(const char* var,vec4 f)
{
    int loc = glGetUniformLocationARB(program,var);
    glUniform4fv(loc,1,f.v);
}

void Shader::setUniform(const char* var,int n,vec4* f)
{
    int loc = glGetUniformLocationARB(program,var);
    glUniform1fvARB(loc,4*n,f->v);
}

void Shader::setUniform(const char* var,mat3 f,bool transpose)
{
    int loc = glGetUniformLocationARB(program,var);
    glUniformMatrix3fvARB(loc,1,transpose,f.mat);
}

void Shader::setUniform(const char* var,int n,mat3* f,bool transpose)
{
    int loc = glGetUniformLocationARB(program,var);
    glUniformMatrix3fvARB(loc,n,transpose,f->mat);
}

void Shader::setUniform(const char* var,mat4 f,bool transpose)
{
    int loc = glGetUniformLocationARB(program,var);
    glUniformMatrix4fvARB(loc,1,transpose,f.mat);
}

void Shader::setUniform(const char* var,int n,mat4* f,bool transpose)
{
    int loc = glGetUniformLocationARB(program,var);
    glUniformMatrix4fvARB(loc,n,transpose,f->mat);
}



