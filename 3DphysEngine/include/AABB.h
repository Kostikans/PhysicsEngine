#pragma once

#include "Transformation.h"
#include "Texture.h"
#include "RigidBody.h"

class AABB : public Transformation
{
private:
	VertexBuffer vbo;
	IndexBuffer ibo;
	VertexAttribBuffer vao;
	Texture* texture;
	glm::vec3 min;
	glm::vec3 max;
	float m_width;
	float m_height;
	float m_depth;

	float m_scale;

	glm::vec3 m_translate;
	glm::quat m_rotate;



public:
	RigidBody* body;
	AABB();
	void draw(Shader& shader, float deltaTime) override;
	void translate(const glm::vec3& translate) override;
	void scale(const float& scale) override;
	void rotate(const glm::quat& rotate) override;
	void init(float width, float height ,float depth);

	void setMass(const float & m_mass);

	void setTexture(std::string path);

	glm::vec3 getMin() const;
	glm::vec3 getMax() const;
	float getWidth() const;
	float getHeight() const;
	float getDepth() const;
	glm::vec3 getPosition() const;


	void move(float deltaTime);
};