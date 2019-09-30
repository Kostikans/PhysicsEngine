#pragma once

#include "Transformation.h"
#include "Texture.h"
#include "RigidBody.h"

class Sphere : public Transformation
{
private:
	VertexBuffer vbo;
	IndexBuffer ibo;
	VertexAttribBuffer vao;
	Texture *texture;
	float m_radius;
	float m_scale;

	glm::vec3 m_translate;
	glm::quat m_rotate;
 
	
public:
	RigidBody* body;
	Sphere();
	void draw(Shader& shader, float deltaTime) override;
	void translate(const glm::vec3& translate) override;
	void scale(const float& scale) override;
	void rotate(const glm::quat& rotate) override;
	void init(float radius);

	void move(float deltaTime);

	float getRadius() const;
	 glm::vec3 getPosition() const;
};