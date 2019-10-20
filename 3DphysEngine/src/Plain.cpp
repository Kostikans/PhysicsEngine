#include "..\include\Plain.h"
#pragma once

Plain::Plain(float mass)
	: m_scale(1.0f)
{
	texture = new Texture("E:\\download\\parket2.0.jpg");
	body = new RigidBody(RIGIDBODY_TYPE_BOX,mass);
}

void Plain::draw(Shader& shader, float deltaTime)
{
	shader.use();
	vao.bind();
	texture->bind();

	AttribLayout layout;
	layout.push<float>(3);
	layout.push<float>(2);
	layout.push<float>(3);
	vao.pushLayout(layout, vbo);

	shader.setInt("texture1", 0);
	glm::mat4x4 modelMatrix = glm::mat4(1.0);
	modelMatrix = glm::translate(modelMatrix, m_translate);
	modelMatrix *= glm::toMat4(m_rotate);
	modelMatrix = glm::scale(modelMatrix, glm::vec3(m_scale, m_scale, m_scale));

	shader.setMat4("modelMatrix", modelMatrix);
	

	ibo.bind();
	glDrawElements(GL_TRIANGLES, ibo.size(), GL_UNSIGNED_INT, 0);
	ibo.unbind();

	
}

void Plain::translate(const glm::vec3& translate)
{
	m_translate += translate;
	body->setPosition(translate);
}

void Plain::scale(const float& scale)
{
	m_scale *= scale;
}

void Plain::rotate(const glm::quat& rotate)
{
	m_rotate *= rotate;
	body->setOrientation(rotate);
	body->update(1.0f/40.0f);
	direction = glm::normalize(glm::vec3(glm::toMat4(m_rotate) * glm::vec4(direction,1.0f)));
}

void Plain::init(float width, float height)
{
	min = glm::vec3(-width, 0.0f, height);
	max = glm::vec3(width, 0.0f, -height);

	m_width = width;
	m_height = height;

	direction = glm::vec3(0.0f, 1.0f, 0.0f);
	std::vector<VertexData> vertexes;

	vertexes.push_back(VertexData(glm::vec3(-width, 0.0f, -height), glm::vec2(0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f)));
	vertexes.push_back(VertexData(glm::vec3(-width, 0.0f,  height), glm::vec2(0.0f, height), glm::vec3(0.0f, 1.0f, 0.0f)));
	vertexes.push_back(VertexData(glm::vec3( width, 0.0f,  height), glm::vec2(width, height), glm::vec3(0.0f, 1.0f, 0.0f)));
	vertexes.push_back(VertexData(glm::vec3( width, 0.0f, -height), glm::vec2(width, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f)));

	std::vector<GLuint> indices = {
		 0, 1, 3,
	     1, 2, 3
	};
	

	vbo.allocate(vertexes.data(), vertexes.size() * sizeof(VertexData));
	ibo.allocate(indices.data(), indices.size() * sizeof(GLuint));	
}

void Plain::move(float deltaTime)
{
	body->update(deltaTime);
}

float Plain::getWidth() const
{
	return m_width;
}

float Plain::getHeight() const
{
	return m_height;
}

glm::vec3 Plain::getDirection() const
{
	return direction;
}

glm::vec3 Plain::getMin() const
{
	return min;
}

glm::vec3 Plain::getMax() const
{
	return max;
}

glm::vec3 Plain::getPosition() const
{
	return body->getPosition();
}
