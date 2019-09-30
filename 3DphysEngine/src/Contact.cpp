#include "..\include\Contact.h"
#include <iostream>

void Contact::setBodyData(RigidBody* one, RigidBody* two, float friction, float restitution)
{
	Contact::body[0] = one;
	Contact::body[1] = two;
	Contact::friction = friction;
	Contact::restitution = restitution;
}

void Contact::calculateInternals(float duration)
{
	calculateContactBasis();


	relativeContactPosition[0] = contactPoint - body[0]->getPosition();
	if (body[1]) {
		relativeContactPosition[1] = contactPoint - body[1]->getPosition();
	}

	contactVelocity = calculateLocalVelocity(0, duration);
	if (body[1]) {
		contactVelocity -= calculateLocalVelocity(1, duration);
	}

	calculateDesiredDeltaVelocity(duration);
}

void Contact::swapBodies()
{
	contactNormal *= -1;

	RigidBody* temp = body[0];
	body[0] = body[1];
	body[1] = temp;
}

void Contact::calculateDesiredDeltaVelocity(float duration)
{
	const static float velocityLimit = 0.25f;

	
	float velocityFromAcc = 0.0f;

	
	velocityFromAcc += glm::dot(body[0]->getAcceleration() , duration * contactNormal);
	if (body[1])
		velocityFromAcc -= glm::dot(body[1]->getAcceleration() , duration * contactNormal);
	
	float thisRestitution = restitution;
	if (abs(contactVelocity.x) < velocityLimit)
	{
		thisRestitution = 0.0f;
	}
	desiredDeltaVelocity = -contactVelocity.x - thisRestitution * (contactVelocity.x - velocityFromAcc);
}

glm::vec3 Contact::calculateLocalVelocity(unsigned bodyIndex, float duration)
{
	RigidBody* thisBody = body[bodyIndex];
	
	glm::vec3 velocity = glm::cross(thisBody->getRotation(),relativeContactPosition[bodyIndex]);
	velocity += thisBody->getVelocity();

	glm::vec3  contactVelocity = glm::transpose(contactToWorld) * velocity ;
	glm::vec3  accVelocity = thisBody->getAcceleration() * duration;

	accVelocity = glm::transpose(contactToWorld) * accVelocity;

	accVelocity.x = 0;

	contactVelocity += accVelocity;

	return contactVelocity;
}

void Contact::calculateContactBasis()
{
	glm::vec3 contactTangent[2];	
	if (fabs(contactNormal.x) > fabs(contactNormal.y))
	{
		const float s = 1.0f / sqrtf(contactNormal.z * contactNormal.z + contactNormal.x * contactNormal.x);

		contactTangent[0].x = contactNormal.z * s;
		contactTangent[0].y = 0;
		contactTangent[0].z = -contactNormal.x * s;
		
		contactTangent[1].x = contactNormal.y * contactTangent[0].x;
		contactTangent[1].y = contactNormal.z * contactTangent[0].x - contactNormal.x * contactTangent[0].z;
		contactTangent[1].z = -contactNormal.y * contactTangent[0].x;
	}
	else
	{
		const float s = 1.0f / sqrtf(contactNormal.z * contactNormal.z + contactNormal.y * contactNormal.y);

		contactTangent[0].x = 0;
		contactTangent[0].y = -contactNormal.z * s;
		contactTangent[0].z = contactNormal.y * s;

		contactTangent[1].x = contactNormal.y * contactTangent[0].z - contactNormal.z * contactTangent[0].y;
		contactTangent[1].y = -contactNormal.x * contactTangent[0].z;
		contactTangent[1].z = contactNormal.x * contactTangent[0].y;
	}

	contactToWorld[0] = contactNormal;
	contactToWorld[1] = contactTangent[0];
	contactToWorld[2] = contactTangent[1];
}

void Contact::applyImpulse(const glm::vec3& impulse, RigidBody* body, glm::vec3* velocityChange, glm::vec3* rotationChange)
{
}

void Contact::applyVelocityChange(glm::vec3 velocityChange[2], glm::vec3 rotationChange[2])
{
	
	glm::mat3x3 inverseInertiaTensor[2];
	glm::vec3 impulseContact;

	if (friction == 0.0f)
	{
		impulseContact = calculateFrictionlessImpulse(inverseInertiaTensor);
	}
	else
	{
		impulseContact = calculateFrictionImpulse(inverseInertiaTensor);
	}

	glm::vec3 impulse = contactToWorld * impulseContact;

	glm::vec3 impulsiveTorque = glm::cross(relativeContactPosition[0], impulse);
	rotationChange[0] = inverseInertiaTensor[0] * impulsiveTorque;
	velocityChange[0] = glm::vec3(0.0f, 0.0f, 0.0f);
	velocityChange[0] += impulse * body[0]->getInverseMass();

	body[0]->addVelocity(velocityChange[0]);
	body[0]->addRotation(rotationChange[0]);

	if (body[1])
	{
		
		glm::vec3 impulsiveTorque = glm::cross(impulse, relativeContactPosition[1]);
		rotationChange[1] = inverseInertiaTensor[1] * impulsiveTorque;
		velocityChange[1] = glm::vec3(0.0f, 0.0f, 0.0f);
		velocityChange[1] += impulse * -body[1]->getInverseMass();

		body[1]->addVelocity(velocityChange[1]);
		body[1]->addRotation(rotationChange[1]);
	}
}

void Contact::applyPositionChange(glm::vec3 linearChange[2], glm::vec3 angularChange[2], float penetration)
{
	const float angularLimit = 0.2f;
	float angularMove[2];
	float linearMove[2];

	float totalInertia = 0.0f;
	float linearInertia[2];
	float angularInertia[2];

	
	for (unsigned i = 0; i < 2; i++) if (body[i])
	{
		glm::mat3x3 inverseInertiaTensor = glm::mat3x3(body[i]->getInvInersiaTensor());

		glm::vec3 angularInertiaWorld = glm::cross(relativeContactPosition[i] , contactNormal);
		angularInertiaWorld = inverseInertiaTensor * angularInertiaWorld;
		angularInertiaWorld = glm::cross(angularInertiaWorld , relativeContactPosition[i]);
		angularInertia[i] = glm::dot(angularInertiaWorld , contactNormal);

		linearInertia[i] = body[i]->getInverseMass();


		totalInertia += linearInertia[i] + angularInertia[i];
	}

	for (unsigned i = 0; i < 2; i++) if (body[i])
	{

		float sign = (i == 0) ? 1 : -1;
		angularMove[i] = sign * penetration * (angularInertia[i] / totalInertia);
		linearMove[i] = sign * penetration * (linearInertia[i] / totalInertia);

	
		glm::vec3 projection = relativeContactPosition[i];
		projection += contactNormal * glm::dot(-relativeContactPosition[i],contactNormal);
	
		float maxMagnitude = angularLimit * glm::length(projection);

		if (angularMove[i] < -maxMagnitude)
		{
			float totalMove = angularMove[i] + linearMove[i];
			angularMove[i] = -maxMagnitude;
			linearMove[i] = totalMove - angularMove[i];
		}
		else if (angularMove[i] > maxMagnitude)
		{
			float totalMove = angularMove[i] + linearMove[i];
			angularMove[i] = maxMagnitude;
			linearMove[i] = totalMove - angularMove[i];
		}


		if (angularMove[i] == 0)
		{
			angularChange[i] = glm::vec3(0.0f, 0.0f, 0.0f);
		}
		else
		{
			glm::vec3 targetAngularDirection = glm::cross(relativeContactPosition[i],contactNormal);

			glm::mat3x3 inverseInertiaTensor = glm::mat3x3(body[i]->getInvInersiaTensor());

			angularChange[i] = inverseInertiaTensor * (targetAngularDirection)  * (angularMove[i] / angularInertia[i]);
		}


		linearChange[i] = contactNormal * linearMove[i];


		glm::vec3 pos = body[i]->getPosition();
		pos += contactNormal* linearMove[i];
		body[i]->setPosition(pos);

		glm::quat q = body[i]->getOrientation();
		q *= glm::angleAxis(1.0f,angularChange[i]);
		body[i]->setOrientation(q);

		body[i]->calculateData();
	}
}

glm::vec3 Contact::calculateFrictionlessImpulse(glm::mat3x3* inverseInertiaTensor)
{
	glm::vec3 impulseContact = glm::vec3(0.0f);

	glm::vec3 deltaVelWorld = glm::cross(relativeContactPosition[0] , contactNormal);
	deltaVelWorld = inverseInertiaTensor[0] * deltaVelWorld;
	deltaVelWorld = glm::cross(deltaVelWorld , relativeContactPosition[0]);

	float deltaVelocity = glm::dot(deltaVelWorld, contactNormal);
	
	deltaVelocity += body[0]->getInverseMass();
	if (body[1])
	{		
		glm::vec3 deltaVelWorld = glm::cross(relativeContactPosition[1] , contactNormal);
		deltaVelWorld = inverseInertiaTensor[1] * deltaVelWorld;
		deltaVelWorld = glm::cross(deltaVelWorld , relativeContactPosition[1]);
		deltaVelocity += glm::dot(deltaVelWorld, contactNormal);
		deltaVelocity += body[1]->getInverseMass();
	}

	impulseContact.x = desiredDeltaVelocity / deltaVelocity;
	impulseContact.y = 0;
	impulseContact.z = 0;
	return impulseContact;
}

glm::vec3 Contact::calculateFrictionImpulse(glm::mat3x3* inverseInertiaTensor)
{
	return glm::vec3();
}

float Contact::computeLambda()
{
	glm::vec3 n1 = contactNormal;
	glm::vec3 w1 = glm::cross(contactNormal, (contactPoint - body[0]->getPosition()));


	glm::vec3 n2 = -contactNormal;
	glm::vec3  w2 = -(glm::cross(contactNormal, (contactPoint - body[1]->getPosition())));


	float initialVelocityProjection =
		glm::dot(n1, body[0]->getVelocity())
		+ glm::dot(w1, body[0]->getRotation())
		+ glm::dot(n2, body[1]->getVelocity())
		+ glm::dot(w2, body[1]->getRotation());


	float 	a = glm::dot(n1, body[0]->getVelocity())
		+ glm::dot(n2, body[1]->getVelocity())
		+ glm::dot(w1, body[0]->getRotation())
		+ glm::dot(w2, body[1]->getRotation());
	+restitution * initialVelocityProjection;


	float b = glm::dot(n1, n1 * body[0]->getInverseMass())
		+ glm::dot(w1, glm::vec3(body[0]->getInvInersiaTensor() * glm::vec4(w1, 1.0f)))
		+ glm::dot(n2, n2 * body[1]->getInverseMass())
		+ glm::dot(w2, glm::vec3(body[1]->getInvInersiaTensor() * glm::vec4(w2, 1.0f)));

	float lambda = -a / b;

	float velocityProjection =
		glm::dot(n1, (body[0]->getVelocity() + n1 * body[0]->getInverseMass() * lambda))
		+ glm::dot(w1, body[0]->getRotation() + w1 * 1.0f * lambda)
		+ glm::dot(n2, (body[1]->getVelocity() + n2 * body[1]->getInverseMass() * lambda))
		+ glm::dot(w2, body[1]->getRotation() + w2 * 1.0f * lambda);

	return lambda;
}



void Contact::aplly(float lambda)
{

	glm::vec3 velocityChange1 = contactNormal * lambda * body[0]->getInverseMass();
	glm::vec3 rotationChange1 = body[0]->getRotation()
		+ glm::cross((contactNormal * lambda) , (contactPoint - body[0]->getPosition())) * body[0]->getInverseMass();

	body[0]->setVelocity(velocityChange1);
	body[0]->addRotation(rotationChange1);

	if (body[1])
	{

		glm::vec3 velocityChange2 = contactNormal * lambda * -body[0]->getInverseMass();
		glm::vec3 rotationChange2 = body[1]->getRotation()
			+ glm::cross((contactNormal * lambda), (contactPoint - body[1]->getPosition())) * body[1]->getInverseMass();

		body[1]->setVelocity(velocityChange2);
		body[1]->addRotation(rotationChange2);
	}
}
