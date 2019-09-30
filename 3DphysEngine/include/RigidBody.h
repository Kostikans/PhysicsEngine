#pragma once

#include "Transformation.h"
#define RIGIDBODY_TYPE_SPHERE 1
#define RIGIDBODY_TYPE_BOX 2

class RigidBody
{
private:
	glm::vec3 position, oldPosition;
	glm::vec3 velocity , forceAccum , torqueAccum;
	glm::vec3 lastFrameAcceleration;
	glm::vec3 acceleration;
	glm::quat orientation;
	glm::vec3 angVel;
	glm::vec3 rotation;

	glm::mat4x4 invTensor;
	glm::mat4x4 modelMatrix;
	float invMass;
	float rest;
	float friction;

public:
	int type;
	RigidBody(int type,float m_mass);
	void update(float deltaTime);
	void setPosition(const glm::vec3 &m_position);
	void AddRotationalImpulse(const glm::vec3& point, const glm::vec3& impulse);
	void AddLinearImpulse(const glm::vec3& impulse);
	glm::vec3 getPosition() const;
	glm::vec3 getVelocity() const;
	glm::vec3 getRotation() const;
	glm::quat getOrientation() const;
	float getInverseMass() const;
	float getFriction() const;
	float getRest() const;
	void setVelocity(const glm::vec3& m_velocity);
	void addTorque(const glm::vec3& m_torque);
	void addForce(const glm::vec3& m_force);

	void addGravity(const glm::vec3& m_gravity);
	void addVelocity(const glm::vec3 &m_velocity);
	void addRotation(const glm::vec3& m_rotation);
	void setOrientation(const glm::quat& m_orientation);
	void setMass(const float& m_mass);
	glm::mat4x4 getInvInersiaTensor() const;
	glm::mat4x4 getModelMatrix() const;

	glm::vec3 getAcceleration() const;
	void clearForces();
	void calculateData();
};