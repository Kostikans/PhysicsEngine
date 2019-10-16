#include "..\include\Contact.h"
#include <iostream>

float Contact::LinearProjectionPercent = 0.4f;
float Contact::PenetrationSlack = 0.01f;
void Contact::setBodyData(RigidBody* one, RigidBody* two, float friction, float restitution)
{
	Contact::body[0] = one;
	Contact::body[1] = two;
	Contact::friction = friction;
	Contact::restitution = restitution;
}

float Contact::computeLambda(int c, glm::vec3 v1, glm::vec3 r1, glm::vec3 v2, glm::vec3 r2)
{
	glm::vec3 n1 = glm::normalize(contactNormal);
	glm::vec3 w1 = glm::cross(contactNormal, (contactPoints[c] - body[0]->getPosition()));

	glm::vec3 n2 = -glm::normalize(contactNormal);
	glm::vec3  w2 = -(glm::cross(contactNormal, (contactPoints[c] - body[1]->getPosition())));

	float initialVelocityProjection =
		glm::dot(n1, v1)
		+ glm::dot(w1, r1)
		+ glm::dot(n2, v2)
		+ glm::dot(w2, r2);

	float a = glm::dot(n1, v1)
		+ glm::dot(n2, v2)
		+ glm::dot(w1, r1)
		+ glm::dot(w2, r2)
	    + restitution * initialVelocityProjection;

	float b = glm::dot(n1, n1 * body[0]->getInverseMass())
		+ glm::dot(w1, w1 * body[0]->getInvInersiaTensor())
		+ glm::dot(n2, n2 * body[1]->getInverseMass())
		+ glm::dot(w2, w2 * body[1]->getInvInersiaTensor());

	float lambda = (-a) / b;
	return lambda;
}



void Contact::aplly(const float &lambda , int c)
{

	glm::vec3 velocityChange1 = body[0]->getVelocity() + contactNormal * lambda* body[0]->getInverseMass();
	glm::vec3 rotationChange1 = body[0]->getRotation() + glm::cross((contactNormal * lambda) , (contactPoints[c] - body[0]->getPosition())) * body[0]->getInvInersiaTensor();
	
	body[0]->setVelocity(velocityChange1);
	body[0]->setRotation(rotationChange1);

	if (body[1]->stat == true)
	{
		glm::vec3 velocityChange2 = body[1]->getVelocity() - contactNormal * lambda * body[1]->getInverseMass();
		glm::vec3 rotationChange2 = body[1]->getRotation() - glm::cross((contactNormal * lambda), (contactPoints[c] - body[1]->getPosition())) * body[1]->getInvInersiaTensor();

		body[1]->setVelocity(velocityChange2);
		body[1]->setRotation(rotationChange2);
	}
}

float Contact::computeLambdaForOne(int c, glm::vec3 v, glm::vec3 r)
{
	glm::vec3 n1 = glm::normalize(contactNormal);
	glm::vec3 w1 = glm::cross(contactNormal, (body[0]->getPosition() - contactPoints[c]));

	float initialVelocityProjection =
		glm::dot(n1, v)
		+ glm::dot(w1, r);

	float a = glm::dot(n1, v)
		+ glm::dot(w1, r)
		+ restitution * initialVelocityProjection;

	float b = glm::dot(n1, n1 * body[0]->getInverseMass())
		+ glm::dot(w1, w1 * body[0]->getInvInersiaTensor());

	float lambda = (-a) / b;
	return lambda;
}

void Contact::applyForOne(const float& lambda , int c)
{
	glm::vec3 velocityChange1 = body[0]->getVelocity() + contactNormal * lambda * body[0]->getInverseMass();
	glm::vec3 rotationChange1 = body[0]->getRotation() + glm::cross((contactNormal * lambda), (body[0]->getPosition() - contactPoints[c])) * body[0]->getInvInersiaTensor();

	body[0]->setVelocity(velocityChange1);
	body[0]->setRotation(rotationChange1);
}

void Contact::resolvePosition()
{
	for (int i = 0; i < 3; ++i)
	{
		float depth = fmaxf(penetration - PenetrationSlack, 0.0f);
		float scalar = penetration / (body[0]->getInverseMass() + body[1]->getInverseMass());
		glm::vec3 correction = contactNormal * scalar * LinearProjectionPercent;
		body[0]->setPosition(body[0]->getPosition() + correction * body[0]->getInverseMass());
		body[1]->setPosition(body[1]->getPosition() - correction * body[1]->getInverseMass());
	}
}

void Contact::resolvePositionForOne()
{
	for (int i = 0; i < 3; ++i)
	{
		float depth = fmaxf(penetration - PenetrationSlack, 0.0f);
		float scalar = penetration / (body[0]->getInverseMass() + body[0]->getInverseMass());
		glm::vec3 correction = contactNormal * scalar * LinearProjectionPercent;
		body[0]->setPosition(body[0]->getPosition() + correction * body[0]->getInverseMass());
	}
	
}

