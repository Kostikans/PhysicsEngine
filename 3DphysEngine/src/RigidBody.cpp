#include "..\include\RigidBody.h"
#include <iostream>

RigidBody::RigidBody(int m_type,float m_mass)
	:rest(0.5f),friction(0.0f),type(m_type),stat(true)
{
	pseudoVelocity = glm::vec3(0.0f);
	pseudoRotation = glm::vec3(0.0f);
	modelMatrix = glm::mat4x4(1.0f);
	acceleration = glm::vec3(0.0f);
	mass = m_mass;
	float ix;
	float iy;
	float iz;
	float iw;
	invTensor = glm::mat3x3(0.0f);
	if (mass != 0 && type == RIGIDBODY_TYPE_SPHERE) {
		float r2 = 1.0f * 1.0f;
		float fraction = (2.0f / 5.0f);
		ix = r2 * mass * fraction;
		iy = r2 * mass * fraction;
		iz = r2 * mass * fraction;

		invTensor[0] = glm::vec3(ix, 0.0f, 0.0f);
		invTensor[1] = glm::vec3(0.0f, iy, 0.0f);
		invTensor[2] = glm::vec3(0.0f, 0.0f, iz);
	}
	else if (mass != 0 && type == RIGIDBODY_TYPE_BOX) {
		glm::vec3 size = glm::vec3(1.0f,1.0f,1.0f) * 2.0f;
		float fraction = (1.0f / 12.0f);
		float x2 = size.x * size.x;
		float y2 = size.y * size.y;
		float z2 = size.z * size.z;
		ix = (y2 + z2) * mass * fraction;
		iy = (x2 + z2) * mass * fraction;
		iz = (x2 + y2) * mass * fraction;


		invTensor[0] = glm::vec3(ix, 0.0f, 0.0f);
		invTensor[1] = glm::vec3(0.0f, iy, 0.0f);
		invTensor[2] = glm::vec3(0.0f, 0.0f, iz);
		
	}
	else if(type == RIGIDBODY_TYPE_PLANE)
		invTensor = glm::mat3x3(0.0f);

	if (mass != 0.0f)
		invTensor = glm::inverse(invTensor) * 0.8f;
	else
		invTensor = glm::mat3x3(0.0f);
	
}

void RigidBody::setStat(bool m_stat)
{
	stat = m_stat;
}

void RigidBody::update(float deltaTime)
{	
	const float damping = 0.97f;

	lastFrameAcceleration = forceAccum * getInverseMass();
	velocity = velocity + lastFrameAcceleration * deltaTime;
	velocity = velocity * damping;

	if (fabsf(velocity.x) < 0.001f) {
		velocity.x = 0.0f;
	}
	if (fabsf(velocity.y) < 0.001f) {
		velocity.y = 0.0f;
	}
	if (fabsf(velocity.z) < 0.001f) {
		velocity.z = 0.0f;
	}

	glm::vec3 angAccel = torqueAccum  * invTensor;
	rotation += angAccel * deltaTime;
	rotation *= damping;

	if (fabsf(rotation.x) < 0.001f) {
		rotation.x = 0.0f;
	}
	if (fabsf(rotation.y) < 0.001f) {
		rotation.y = 0.0f;
	}
	if (fabsf(rotation.z) < 0.001f) {
		rotation.z = 0.0f;
	}
	
	glm::quat temp;
	temp.w = 0.0f;
	temp.x = rotation.x * deltaTime;
	temp.y = rotation.y * deltaTime;
	temp.z = rotation.z * deltaTime;
	temp *= orientation;
	temp *= 0.5f;
	orientation += temp;	

	position += velocity * deltaTime ;
	position += pseudoVelocity * deltaTime;

	calculateData();
	clearForces();
}
	
void RigidBody::setPosition(const glm::vec3 &m_position)
{
	position = m_position;
}

void RigidBody::AddRotationalImpulse(const glm::vec3& point, const glm::vec3& impulse)
{
	glm::vec3 centerOfMass = position;
	glm::vec3 torque = glm::cross(point - centerOfMass, impulse);
	torqueAccum += invTensor * torque;
}


void RigidBody::AddLinearImpulse(const glm::vec3& impulse)
{
	velocity = velocity + impulse;
}

glm::vec3 RigidBody::getPosition() const
{
	return position;
}

glm::vec3 RigidBody::getVelocity() const
{
	return velocity;
}

glm::vec3 RigidBody::getRotation() const
{
	return rotation;
}

glm::quat RigidBody::getOrientation() const
{
	return orientation;
}

void RigidBody::setRotation(const glm::vec3& m_rotate)
{
	rotation = m_rotate;
}

float RigidBody::getInverseMass() const
{
	if (mass == 0.0f)
		return 0.0f;
	else
		return 1.0f/ mass;
}

float RigidBody::getFriction() const
{
	return friction;
}

float RigidBody::getRest() const
{
	return rest;
}

float RigidBody::getMass() const
{
	return mass;
}

void RigidBody::setVelocity(const glm::vec3& m_velocity)
{
	velocity = m_velocity;
}

void RigidBody::setPseudoVelocity(const glm::vec3& m_velocity)
{
	pseudoVelocity += m_velocity;
}

void RigidBody::setPseudoRotation(const glm::vec3& m_rotate)
{
	pseudoRotation += m_rotate;
}

void RigidBody::addTorque(const glm::vec3& m_torque)
{
	torqueAccum += m_torque;
}

void RigidBody::addForce(const glm::vec3& m_force)
{
	forceAccum += m_force ;
}

void RigidBody::addGravity(const glm::vec3& m_gravity)
{
	forceAccum += m_gravity;
}

void RigidBody::addVelocity(const glm::vec3& m_velocity)
{
	velocity += m_velocity;
}

void RigidBody::addRotation(const glm::vec3& m_rotation)
{
	rotation += m_rotation;
}

void RigidBody::setOrientation(const glm::quat& m_orientation)
{
	orientation = m_orientation;
}

void RigidBody::setMass(const float& m_mass)
{
	mass = m_mass;
}

glm::mat3x3 RigidBody::getInvInersiaTensor() const
{
	return invTensor;
}

glm::mat4x4 RigidBody::getModelMatrix() const
{
	return modelMatrix;
}

glm::vec3 RigidBody::getAcceleration() const
{
	return lastFrameAcceleration;
}

void RigidBody::clearForces()
{
	forceAccum = glm::vec3(0.0f);
	torqueAccum = glm::vec3(0.0f);
	pseudoVelocity = glm::vec3(0.0f);
	pseudoRotation = glm::vec3(0.0f);
}

void RigidBody::calculateData() 
{
	modelMatrix = glm::mat4x4(1.0f);
	orientation = glm::normalize(orientation);


	modelMatrix = glm::translate(modelMatrix, position);
	modelMatrix *= glm::toMat4(orientation);
}
