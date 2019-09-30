#include "..\include\RigidBody.h"

RigidBody::RigidBody(int m_type,float m_mass)
	:rest(0.5f),friction(0.0f),type(m_type)
{
	acceleration = glm::vec3(0.0f);
	invMass = m_mass;
	float ix;
	float iy;
	float iz;
	float iw;
	invTensor = glm::mat4x4(0.0f);
	if (invMass != 0 && type == RIGIDBODY_TYPE_SPHERE) {
		float r2 = 0.5f * 0.5f;
		float fraction = (2.0f / 5.0f);
		ix = r2 * invMass * fraction;
		iy = r2 * invMass * fraction;
		iz = r2 * invMass * fraction;
		iw = 1.0f;

		invTensor[0] = glm::vec4(ix, 0.0f, 0.0f, 0.0f);
		invTensor[1] = glm::vec4(0.0f, iy, 0.0f, 0.0f);
		invTensor[2] = glm::vec4(0.0f, 0.0f, iz, 0.0f);
		invTensor[3] = glm::vec4(0.0f, 0.0f, 0.0f, iw);
	}
	else if (invMass != 0 && type == RIGIDBODY_TYPE_BOX) {
		glm::vec3 size = glm::vec3(1.0f,1.0f,1.0f) * 2.0f;
		float fraction = (1.0f / 12.0f);
		float x2 = 1.0f;
		float y2 = 1.0f;
		float z2 = 1.0f;
		ix = (y2 + z2) * invMass * fraction;
		iy = (x2 + z2) * invMass * fraction;
		iz = (x2 + y2) * invMass * fraction;
		iw = 1.0f;

		invTensor[0] = glm::vec4(ix, 0.0f, 0.0f, 0.0f);
		invTensor[1] = glm::vec4(0.0f, iy, 0.0f, 0.0f);
		invTensor[2] = glm::vec4(0.0f, 0.0f, iz, 0.0f);
		invTensor[3] = glm::vec4(0.0f, 0.0f, 0.0f, iw);
	}

	
}

void RigidBody::update(float deltaTime)
{
	const float damping = 0.98f;
	lastFrameAcceleration = acceleration;
	lastFrameAcceleration += forceAccum ;
	velocity = velocity + lastFrameAcceleration * deltaTime;
	velocity = velocity * damping;

	glm::vec3 angAccel = glm::vec3(glm::vec4(torqueAccum,1.0f)  * invTensor);
	rotation += angAccel * deltaTime;
	rotation *= damping;

	glm::quat kek;
	kek.w = 0.0f;
	kek.x = rotation.x * deltaTime;
	kek.y = rotation.y * deltaTime;
	kek.z = rotation.z * deltaTime;
	kek *= orientation;
	orientation += kek;

	position = position + velocity * deltaTime;
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
	glm::vec4 angAccel = invTensor * glm::vec4(torque,1.0f);
	torqueAccum += glm::vec3(angAccel);
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

float RigidBody::getInverseMass() const
{
	return invMass;
}

float RigidBody::getFriction() const
{
	return friction;
}

float RigidBody::getRest() const
{
	return rest;
}

void RigidBody::setVelocity(const glm::vec3& m_velocity)
{
	velocity = m_velocity;
}

void RigidBody::addTorque(const glm::vec3& m_torque)
{
	torqueAccum += m_torque;
}

void RigidBody::addForce(const glm::vec3& m_force)
{
	forceAccum += m_force * invMass;
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
	invMass = m_mass;
}

glm::mat4x4 RigidBody::getInvInersiaTensor() const
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
}

void RigidBody::calculateData() 
{
	modelMatrix = glm::mat4x4(1.0f);
	orientation = glm::normalize(orientation);
	modelMatrix = glm::translate(modelMatrix, position);
	modelMatrix *= glm::toMat4(orientation);
}
