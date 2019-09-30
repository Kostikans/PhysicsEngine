#pragma once

#define GLEW_STATIC
#include <GL/glew.h>

class Texture
{
private:
	GLuint ID;
public:
	Texture();
	Texture(const char* texturePath);
	void allocate(const char* texturePath);
	void bind();
	void unbind();
};