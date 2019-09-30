#pragma once

#include "Transformation.h"
#include "RigidBody.h"

const glm::vec3 GRAVITY = glm::vec3(0.0f, -9.81f, 0.0f);

class Gravity
{
private:
	glm::vec3 gravity;
public:
	Gravity() {
		gravity = GRAVITY;
	}
	void setGravity(const glm::vec3& m_gravity)
	{
		gravity = m_gravity;
	}
	void updateGravity(RigidBody* body)
	{
		body->addGravity(gravity);
	}
};