#include "..\include\VertexBuffer.h"

#include <iostream>
VertexBuffer::VertexBuffer()
{
}
VertexBuffer::VertexBuffer(const float *m_data, GLsizei size)
{
	glGenBuffers(1, &ID);
	glBindBuffer(GL_ARRAY_BUFFER, ID);
	glBufferData(GL_ARRAY_BUFFER, size , m_data, GL_STATIC_DRAW);
}

void VertexBuffer::allocate(const void* m_data, GLsizei size)
{
	glGenBuffers(1, &ID);
	glBindBuffer(GL_ARRAY_BUFFER, ID);
	glBufferData(GL_ARRAY_BUFFER, size, m_data, GL_STATIC_DRAW);
	
}

void VertexBuffer::bind() const
{
	glBindBuffer(GL_ARRAY_BUFFER, ID);
}

void VertexBuffer::unbind() const
{
	glBindBuffer(GL_ARRAY_BUFFER, 0);
}
