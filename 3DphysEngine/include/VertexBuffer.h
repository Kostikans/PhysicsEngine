#pragma once

#define GLEW_STATIC
#include <GL/glew.h>


class VertexBuffer
{
private:
	GLuint ID;

public:
	VertexBuffer();
	VertexBuffer(const float *m_data ,GLsizei size);
	void allocate(const void* m_data, GLsizei size);
	void bind() const;
	void unbind() const;
};
