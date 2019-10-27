#pragma once

#include "Transformation.h"
#include "Texture.h"
#include "Gravity.h"


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
 
	int type;
	Gravity gravity;
public:

	RigidBody* body;
	Sphere(float mass);

	void draw(Shader& shader, float deltaTime) override;
	void translate(const glm::vec3& translate) override;
	void scale(const float& scale) override;
	void rotate(const glm::quat& rotate) override;
	void move(float deltaTime) override;
	int getType() override;
	void updateGravity(float deltaTime) override;
	void init(float radius, float crutch = 0, float depth = 0) override;
	void drawNormal(Shader& shader) override;
	void addImpulse(const glm::vec3& impulse) override;
	void addTorque(const glm::vec3& impulse)  override;

	float getRadius() const;
	glm::vec3 getPosition() const;
};