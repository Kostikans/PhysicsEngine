#pragma once

#include "Transformation.h"


class RigidBody
{
private:
	glm::vec3 position;
	glm::vec3 pseudoVelocity , velocity , forceAccum , torqueAccum;
	glm::vec3 lastFrameAcceleration;
	glm::vec3 acceleration;
	glm::quat orientation;
	glm::vec3 angVel;
	glm::vec3 rotation;
	glm::vec3 pseudoRotation;

	glm::mat3x3 invTensor;
	glm::mat4x4 modelMatrix;

	float mass;
	float rest;
	float friction;
	
public:
	int type;
	bool stat;
	RigidBody(int type,float m_mass);
	void setStat(bool m_stat);
	void update(float deltaTime);
	void setPosition(const glm::vec3 &m_position);
	void AddRotationalImpulse(const glm::vec3& point, const glm::vec3& impulse);
	void AddLinearImpulse(const glm::vec3& impulse);
	glm::vec3 getPosition() const;
	glm::vec3 getVelocity() const;
	glm::vec3 getRotation() const;
	glm::quat getOrientation() const;
	void setRotation(const glm::vec3& m_rotate);
	float getInverseMass() const;
	float getFriction() const;
	float getRest() const;
	float getMass() const;
	void setVelocity(const glm::vec3& m_velocity);
	void setPseudoVelocity(const glm::vec3& m_velocity);
	void setPseudoRotation(const glm::vec3& m_rotate);
	void addTorque(const glm::vec3& m_torque);
	void addForce(const glm::vec3& m_force);

	void addGravity(const glm::vec3& m_gravity);
	void addVelocity(const glm::vec3 &m_velocity);
	void addRotation(const glm::vec3& m_rotation);
	void setOrientation(const glm::quat& m_orientation);
	void setMass(const float& m_mass);
	glm::mat3x3 getInvInersiaTensor() const;
	glm::mat4x4 getModelMatrix() const;

	glm::vec3 getAcceleration() const;
	void clearForces();
	void calculateData();
};