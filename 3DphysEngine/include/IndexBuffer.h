#pragma once

#define GLEW_STATIC
#include <GL/glew.h>

class IndexBuffer
{
private:
	GLuint ID;
	GLsizei m_size;
public:
	IndexBuffer();
	IndexBuffer(const unsigned int* m_data, GLsizei size);
	void allocate(const void* m_data, GLsizei size);
	GLsizei size();
	void bind() const;
	void unbind() const;
};