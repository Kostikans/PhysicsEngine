#include <iostream>
#include "..\include\AABB.h"

AABB::AABB(float mass)
	: m_scale(1.0f)
{
	texture = new Texture("E:\\download\\8640003215_50cc68f8cf_b.jpg");
	body = new RigidBody(RIGIDBODY_TYPE_BOX,mass);
}

void AABB::drawNormal(Shader& shader)
{
	shader.use();
	

	AttribLayout layout1;
	layout1.push<float>(3);
	normalVao.pushLayout(layout1, normalVbo);

	glm::vec3 xAxis = glm::vec3(1.0f, 0.0f, 0.0f);
	glm::vec3 yAxis = glm::vec3(0.0f, 1.0f, 0.0f);
	glm::vec3 zAxis = glm::vec3(0.0f, 0.0f, 1.0f);

	glm::mat4x4 model = glm::mat4x4(1.0f);
	model = glm::translate(model, m_translate);
	shader.setMat4("modelMatrix", model);
	shader.setVec3("AxisColor",xAxis);
	
	normalVbo.bind();
	glLineWidth((GLfloat)LINE_WIDTH);
	glDrawArrays(GL_LINES, 0, 2);
	shader.setVec3("AxisColor", yAxis);
	glDrawArrays(GL_LINES, 2, 2);
	shader.setVec3("AxisColor", zAxis);
	glDrawArrays(GL_LINES, 4, 2);
	normalVbo.unbind();
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
	m_width = width;
	m_height = height;
	m_depth = depth;

	m_normX = glm::vec3(1.0f, 0.0f, 0.0f);
	m_normY = glm::vec3(0.0f, 1.0f, 0.0f);
	m_normZ = glm::vec3(0.0f, 0.0f, 1.0f);
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

	std::vector<glm::vec3> normals;

	normals.push_back(glm::vec3(0.0f));
	normals.push_back(m_normX * LINE_WIDTH);
	normals.push_back(glm::vec3(0.0f));
	normals.push_back(m_normY * LINE_WIDTH);
	normals.push_back(glm::vec3(0.0f));
	normals.push_back(m_normZ * LINE_WIDTH);
	

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

	normalVbo.allocate(normals.data(), normals.size() * sizeof(glm::vec3));
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

glm::vec3 AABB::getNormX() const
{
	return m_normX;
}

glm::vec3 AABB::getNormY() const
{
	return m_normY;
}

glm::vec3 AABB::getNormZ() const
{
	return m_normZ;
}

glm::vec3 AABB::getAxis(int index) const
{
	if (index == 0)
		return m_normX;
	if (index == 1)
		return m_normY;
	if (index == 2)
		return m_normZ;
}

float AABB::halfsize(int index) const
{
	if (index == 0)
		return m_width;
	if (index == 1)
		return m_height;
	if (index == 2)
		return m_depth;
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
	m_translate = body->getPosition();
	glm::vec3 normx = glm::vec3((glm::toMat4(body->getOrientation())) * glm::vec4(1.0f,0.0f,0.0f, 1.0f));
	glm::vec3 normy = glm::vec3((glm::toMat4(body->getOrientation())) * glm::vec4(0.0f, 1.0f, 0.0f, 1.0f));
	glm::vec3 normz = glm::vec3((glm::toMat4(body->getOrientation())) * glm::vec4(0.0f, 0.0f, 1.0f, 1.0f));

	std::vector<glm::vec3> normals;

	normals.push_back(glm::vec3(0.0f));
	normals.push_back(normx * LINE_WIDTH);
	normals.push_back(glm::vec3(0.0f));
	normals.push_back(normy * LINE_WIDTH);
	normals.push_back(glm::vec3(0.0f));
	normals.push_back(normz * LINE_WIDTH);
	normalVbo.allocate(normals.data(), normals.size() * sizeof(glm::vec3));

	m_normX = normx;
	m_normY = normy;
	m_normZ = normz;

	
}

glm::vec3 AABB::getMin() const
{
	return min;
}
