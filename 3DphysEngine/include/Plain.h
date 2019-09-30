#pragma once

#include "Transformation.h"
#include "Texture.h"
#include "RigidBody.h"

class Plain : public Transformation
{
private:
	VertexBuffer vbo;
	IndexBuffer ibo;
	VertexAttribBuffer vao;
	Texture* texture;

	glm::vec3 min;
	glm::vec3 max;
	glm::vec3 position; 
	glm::vec3 direction;

	glm::vec3 m_translate;
	glm::quat m_rotate;
	float m_scale;
	

public:
	RigidBody* body;
	Plain();
	void draw(Shader& shader, float deltaTime) override;
	void translate(const glm::vec3& translate) override;
	void scale(const float& scale) override;
	void rotate(const glm::quat& rotate) override;
	void init(float widht,float height);

	glm::vec3 getDirection() const;
	glm::vec3 getMin() const;
	glm::vec3 getMax() const;
	glm::vec3 getPosition() const;
};