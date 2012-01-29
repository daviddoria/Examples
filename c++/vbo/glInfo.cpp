///////////////////////////////////////////////////////////////////////////////
// glInfo.cpp
// ==========
// get GL vendor, version, supported extensions and other states using glGet*
// functions and store them glInfo struct variable
//
// To get valid OpenGL infos, OpenGL rendering context (RC) must be opened 
// before calling glInfo::getInfo(). Otherwise it returns false.
//
//  AUTHOR: Song Ho Ahn (song.ahn@gmail.com)
// CREATED: 2005-10-04
// UPDATED: 2006-12-14
//
// Copyright (c) 2005 Song Ho Ahn
///////////////////////////////////////////////////////////////////////////////

#include <GL/gl.h>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include "glInfo.h"
using std::string;
using std::stringstream;
using std::vector;
using std::cout;
using std::endl;



///////////////////////////////////////////////////////////////////////////////
// extract openGL info
// This function must be called after GL rendering context opened.
///////////////////////////////////////////////////////////////////////////////
bool glInfo::getInfo()
{
    char* str = 0;
    char* tok = 0;

    // get vendor string
    str = (char*)glGetString(GL_VENDOR);
    if(str) this->vendor = str;                  // check NULL return value
    else return false;

    // get renderer string
    str = (char*)glGetString(GL_RENDERER);
    if(str) this->renderer = str;                // check NULL return value
    else return false;

    // get version string
    str = (char*)glGetString(GL_VERSION);
    if(str) this->version = str;                 // check NULL return value
    else return false;

    // get all extensions as a string
    str = (char*)glGetString(GL_EXTENSIONS);

    // split extensions
    if(str)
    {
	    /*
        tok = strtok((char*)str, " ");
        while(tok)
        {
            this->extensions.push_back(tok);    // put a extension into struct
            tok = strtok(0, " ");               // next token
        }
	    */
    }
    else
    {
		return false;
	}
		
    // sort extension by alphabetical order
    std::sort(this->extensions.begin(), this->extensions.end());

    // get number of color bits
    glGetIntegerv(GL_RED_BITS, &this->redBits);
    glGetIntegerv(GL_GREEN_BITS, &this->greenBits);
    glGetIntegerv(GL_BLUE_BITS, &this->blueBits);
    glGetIntegerv(GL_ALPHA_BITS, &this->alphaBits);

    // get depth bits
    glGetIntegerv(GL_DEPTH_BITS, &this->depthBits);

    // get stecil bits
    glGetIntegerv(GL_STENCIL_BITS, &this->stencilBits);

    // get max number of lights allowed
    glGetIntegerv(GL_MAX_LIGHTS, &this->maxLights);

    // get max texture resolution
    glGetIntegerv(GL_MAX_TEXTURE_SIZE, &this->maxTextureSize);

    // get max number of clipping planes
    glGetIntegerv(GL_MAX_CLIP_PLANES, &this->maxClipPlanes);

    // get max modelview and projection matrix stacks
    glGetIntegerv(GL_MAX_MODELVIEW_STACK_DEPTH, &this->maxModelViewStacks);
    glGetIntegerv(GL_MAX_PROJECTION_STACK_DEPTH, &this->maxProjectionStacks);
    glGetIntegerv(GL_MAX_ATTRIB_STACK_DEPTH, &this->maxAttribStacks);

    // get max texture stacks
    glGetIntegerv(GL_MAX_TEXTURE_STACK_DEPTH, &this->maxTextureStacks);

    return true;
}



///////////////////////////////////////////////////////////////////////////////
// check if the video card support a certain extension
///////////////////////////////////////////////////////////////////////////////
bool glInfo::isExtensionSupported(const char* ext)
{
    // search corresponding extension
    std::vector< string >::const_iterator iter = this->extensions.begin();
    std::vector< string >::const_iterator endIter = this->extensions.end();

    while(iter != endIter)
    {
        if(ext == *iter)
            return true;
        else
            ++iter;
    }
    return false;
}



///////////////////////////////////////////////////////////////////////////////
// print OpenGL info to screen and save to a file
///////////////////////////////////////////////////////////////////////////////
void glInfo::printSelf()
{
    stringstream ss;

    ss << endl; // blank line
    ss << "OpenGL Driver Info" << endl;
    ss << "==================" << endl;
    ss << "Vendor: " << this->vendor << endl;
    ss << "Version: " << this->version << endl;
    ss << "Renderer: " << this->renderer << endl;

    ss << endl;
    ss << "Color Bits(R,G,B,A): (" << this->redBits << ", " << this->greenBits
       << ", " << this->blueBits << ", " << this->alphaBits << ")\n";
    ss << "Depth Bits: " << this->depthBits << endl;
    ss << "Stencil Bits: " << this->stencilBits << endl;

    ss << endl;
    ss << "Max Texture Size: " << this->maxTextureSize << "x" << this->maxTextureSize << endl;
    ss << "Max Lights: " << this->maxLights << endl;
    ss << "Max Clip Planes: " << this->maxClipPlanes << endl;
    ss << "Max Modelview Matrix Stacks: " << this->maxModelViewStacks << endl;
    ss << "Max Projection Matrix Stacks: " << this->maxProjectionStacks << endl;
    ss << "Max Attribute Stacks: " << this->maxAttribStacks << endl;
    ss << "Max Texture Stacks: " << this->maxTextureStacks << endl;

    ss << endl;
    ss << "Total Number of Extensions: " << this->extensions.size() << endl;
    ss << "==============================" << endl;
    for(unsigned int i = 0; i < this->extensions.size(); ++i)
        ss << this->extensions.at(i) << endl;

    ss << "======================================================================" << endl;

    cout << ss.str() << endl;
}
