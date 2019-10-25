#include "..\include\IndexBuffer.h"

IndexBuffer::IndexBuffer()
{
}

IndexBuffer::IndexBuffer(const unsigned int* m_data , GLsizei size)
{
	m_size = size;
	glGenBuffers(1, &ID);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ID);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, size, m_data, GL_STATIC_DRAW);
}

void IndexBuffer::allocate(const void * m_data, GLsizei size)
{
	m_size = size;
	glGenBuffers(1, &ID);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ID);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, size, m_data, GL_STATIC_DRAW);
}

GLsizei IndexBuffer::size()
{
	return m_size;
}

void IndexBuffer::bind() const
{
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ID);
}

void IndexBuffer::unbind() const
{
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}
