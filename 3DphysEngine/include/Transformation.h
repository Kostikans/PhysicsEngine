#pragma once

#include <vector>
#include "VertexBuffer.h"
#include "IndexBuffer.h"
#include "AttribLayout.h"
#include "VertexAttribBuffer.h"
#include "Shader.h"
#include "glm.hpp"
#include "gtx/quaternion.hpp"
#include "mat4x4.hpp"
#include "gtc/matrix_transform.hpp"
#include "RigidBody.h"


struct VertexData
{
	VertexData() {};
	VertexData(glm::vec3 m_pos, glm::vec2 m_texCoord, glm::vec3 m_norm)
	{
		position = m_pos;
		texCoord = m_texCoord;
		normal = m_norm;
	}
	VertexData(glm::vec3 m_pos)
	{
		position = m_pos;
	}
	glm::vec3 position;
	glm::vec2 texCoord;
	glm::vec3 normal;
};


class Transformation
{
public:
	virtual void draw(Shader& shader,float deltaTime) = 0;
	virtual void translate(const glm::vec3 &translate) = 0;
	virtual void scale(const float& scale) = 0;
    virtual void rotate(const glm::quat& rotate) = 0;
};