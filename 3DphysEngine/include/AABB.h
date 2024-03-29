#pragma once

#include "Transformation.h"
#include "Texture.h"
#include "Gravity.h"

const float LINE_WIDTH = 2.0f;

class AABB : public Transformation
{
private:
	VertexBuffer vbo;
	VertexBuffer normalVbo;
	IndexBuffer ibo;
	VertexAttribBuffer vao;
	VertexAttribBuffer normalVao;
	Texture* texture;
	glm::vec3 min;
	glm::vec3 max;
	float m_width;
	float m_height;
	float m_depth;


	glm::vec3 m_normX;
	glm::vec3 m_normY;
	glm::vec3 m_normZ;
	float m_scale;

	glm::vec3 m_translate;
	glm::quat m_rotate;

	int type;
	Gravity gravity;
public:

	RigidBody* body;
	AABB(float mass);
	AABB();
	void drawNormal(Shader& shader);
	void draw(Shader& shader, float deltaTime) override;
	void translate(const glm::vec3& translate) override;
	void scale(const float& scale) override;
	void rotate(const glm::quat& rotate) override;
	void move(float deltaTime) override;
	int getType() override;
	void updateGravity(float deltaTime) override;
	void init(float width, float height ,float depth) override;
	void addImpulse(const glm::vec3& impulse) override;
	void addTorque(const glm::vec3& impulse)  override;
	void setMass(const float & m_mass);

	void setTexture(std::string path);

	glm::vec3 getMin() const;
	glm::vec3 getMax() const;
	glm::vec3 getNormX() const;
	glm::vec3 getNormY() const;
	glm::vec3 getNormZ() const;

	glm::vec3 getAxis(int index) const;
	float halfsize(int index) const;
	float getWidth() const;
	float getHeight() const;
	float getDepth() const;
	glm::vec3 getPosition() const;
	
	glm::vec3 halfSize() const;
};