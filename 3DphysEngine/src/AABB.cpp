#include <iostream>
#include "..\include\AABB.h"

AABB::AABB()
	: m_scale(1.0f)
{
	texture = new Texture("E:\\download\\8640003215_50cc68f8cf_b.jpg");
	body = new RigidBody(RIGIDBODY_TYPE_BOX,1.0f);
}

void AABB::draw(Shader& shader, float deltaTime)
{
	shader.use();
	vao.bind();
	texture->bind();
	shader.setInt("texture1", 0);

	AttribLayout layout;
	layout.push<float>(3);
	layout.push<float>(2);
	layout.push<float>(3);
	vao.pushLayout(layout, vbo);

	shader.setMat4("modelMatrix", body->getModelMatrix());

	ibo.bind(); 
	glDrawElements(GL_TRIANGLES, ibo.size(), GL_UNSIGNED_INT, 0);
	ibo.unbind();
}

void AABB::translate(const glm::vec3& translate)
{
	m_translate += translate;
	body->setPosition(m_translate);
	
}

void AABB::scale(const float& scale)
{
	m_scale *= scale;
}

void AABB::rotate(const glm::quat& rotate)
{
	m_rotate *= rotate;
}

void AABB::init(float width, float height, float depth)
{

	width = 1.0f;
	height = 1.0f;
	depth = 1.0f;
	m_width = width;
	m_height = height;
	m_depth = depth;
	std::vector<VertexData> vertexes;

	vertexes.push_back(VertexData(glm::vec3(-width, height, depth), glm::vec2(0.0f, height), glm::vec3(0.0f, 0.0f, 1.0f)));
	vertexes.push_back(VertexData(glm::vec3(-width, -height, depth), glm::vec2(0.0f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f)));
	vertexes.push_back(VertexData(glm::vec3(width, height, depth), glm::vec2(width, height), glm::vec3(0.0f, 0.0f, 1.0f)));
	vertexes.push_back(VertexData(glm::vec3(width, -height, depth), glm::vec2(width, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f)));

	vertexes.push_back(VertexData(glm::vec3(width, height, depth), glm::vec2(0.0f, height), glm::vec3(1.0f, 0.0f, 0.0f)));
	vertexes.push_back(VertexData(glm::vec3(width, -height, depth), glm::vec2(0.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f)));
	vertexes.push_back(VertexData(glm::vec3(width, height, -depth), glm::vec2(depth, height), glm::vec3(1.0f, 0.0f, 0.0f)));
	vertexes.push_back(VertexData(glm::vec3(width, -height, -depth), glm::vec2(depth, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f)));

	vertexes.push_back(VertexData(glm::vec3(width, height, depth), glm::vec2(0.0f, width), glm::vec3(0.0f, 1.0f, 0.0f)));
	vertexes.push_back(VertexData(glm::vec3(width, height, -depth), glm::vec2(0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f)));
	vertexes.push_back(VertexData(glm::vec3(-width, height, depth), glm::vec2(width, width), glm::vec3(0.0f, 1.0f, 0.0f)));
	vertexes.push_back(VertexData(glm::vec3(-width, height, -depth), glm::vec2(width, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f)));

	vertexes.push_back(VertexData(glm::vec3(width, height, -depth), glm::vec2(0.0f, height), glm::vec3(0.0f, 0.0f, -1.0f)));
	vertexes.push_back(VertexData(glm::vec3(width, -height, -depth), glm::vec2(0.0f, 0.0f), glm::vec3(0.0f, 0.0f, -1.0f)));
	vertexes.push_back(VertexData(glm::vec3(-width, height, -depth), glm::vec2(width, height), glm::vec3(0.0f, 0.0f,-1.0f)));
	vertexes.push_back(VertexData(glm::vec3(-width, -height, -depth), glm::vec2(width, 0.0f), glm::vec3(0.0f, 0.0f, -1.0f)));

	vertexes.push_back(VertexData(glm::vec3(-width, height, depth), glm::vec2(depth, height), glm::vec3(-1.0f, 0.0f, 0.0f)));
	vertexes.push_back(VertexData(glm::vec3(-width, height, -depth), glm::vec2(0.0f, height), glm::vec3(-1.0f, 0.0f, 0.0f)));
	vertexes.push_back(VertexData(glm::vec3(-width, -height, depth), glm::vec2(depth, 0.0f), glm::vec3(-1.0f, 0.0f, 0.0f)));
	vertexes.push_back(VertexData(glm::vec3(-width, -height, -depth), glm::vec2(0.0f, 0.0f), glm::vec3(-1.0f, 0.0f, 0.0f)));

	vertexes.push_back(VertexData(glm::vec3(-width, -height, depth), glm::vec2(0.0f, height), glm::vec3(0.0f, -1.0f, 0.0f)));
	vertexes.push_back(VertexData(glm::vec3(-width, -height, -depth), glm::vec2(0.0f, 0.0f), glm::vec3(0.0f, -1.0f, 0.0f)));
	vertexes.push_back(VertexData(glm::vec3(width, -height, depth), glm::vec2(width, height), glm::vec3(0.0f, -1.0f, 0.0f)));
	vertexes.push_back(VertexData(glm::vec3(width, -height, -depth), glm::vec2(width, 0.0f), glm::vec3(0.0f, -1.0f, 0.0f)));


	std::vector<GLuint> indices;
	for (int i = 0; i < 24; i += 4) {
		indices.push_back(i + 0);
		indices.push_back(i + 1);
		indices.push_back(i + 2);
		indices.push_back(i + 2);
		indices.push_back(i + 1);
		indices.push_back(i + 3);
	}


	vbo.allocate(vertexes.data(), vertexes.size() * sizeof(VertexData));
	ibo.allocate(indices.data(), indices.size() * sizeof(GLuint));
}

void AABB::setMass(const float & m_mass)
{
	body->setMass(m_mass);
}

void AABB::setTexture(std::string path)
{
	delete texture;
	texture = new Texture(path.c_str());
}

glm::vec3 AABB::getMax() const
{
	return max;
}

float AABB::getWidth() const
{
	return m_width;
}

float AABB::getHeight() const
{
	return m_height;
}

float AABB::getDepth() const
{
	return m_depth;
}

glm::vec3 AABB::getPosition() const
{
	return body->getPosition();
}

glm::vec3 AABB::halfSize() const
{
	return glm::vec3(m_width,m_height,m_depth);
}

void AABB::move(float deltaTime)
{
	body->update(deltaTime);
}

glm::vec3 AABB::getMin() const
{
	return min;
}
