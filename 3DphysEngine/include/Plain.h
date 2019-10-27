#pragma once

#include "Transformation.h"
#include "Texture.h"


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
	
	float m_width;
	float m_height;

	int type;
public:
	RigidBody* body;
	Plain(float mass);
	void draw(Shader& shader, float deltaTime) override;
	void translate(const glm::vec3& translate) override;
	void scale(const float& scale) override;
	void rotate(const glm::quat& rotate) override;
	void move(float deltaTime) override;
	void updateGravity(float deltaTime) override;
	int getType() override;
	void init(float widht,float height , float crutch = 0) override;
	void drawNormal(Shader& shader) override;
	void addImpulse(const glm::vec3& impulse) override;
	void addTorque(const glm::vec3& impulse)  override;

	float getWidth() const;
	float getHeight() const;
	glm::vec3 getDirection() const;
	glm::vec3 getMin() const;
	glm::vec3 getMax() const;
	glm::vec3 getPosition() const;
};