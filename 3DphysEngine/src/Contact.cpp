#include "..\include\Contact.h"
#include <iostream>

void Contact::setBodyData(RigidBody* one, RigidBody* two, float friction, float restitution)
{
	Contact::body[0] = one;
	Contact::body[1] = two;
	Contact::friction = friction;
	Contact::restitution = restitution;
}

float Contact::computeLambda()
{
	glm::vec3 n1 = glm::normalize(contactNormal);
	glm::vec3 w1 = glm::cross(contactNormal, (contactPoint - body[0]->getPosition()));

	glm::vec3 n2 = -glm::normalize(contactNormal);
	glm::vec3  w2 = -(glm::cross(contactNormal, (contactPoint - body[1]->getPosition())));

	float initialVelocityProjection =
		glm::dot(n1, body[0]->getVelocity())
		+ glm::dot(w1, body[0]->getRotation())
		+ glm::dot(n2, body[1]->getVelocity())
		+ glm::dot(w2, body[1]->getRotation());

	float a = glm::dot(n1, body[0]->getVelocity())
		+ glm::dot(n2, body[1]->getVelocity())
		+ glm::dot(w1, body[0]->getRotation())
		+ glm::dot(w2, body[1]->getRotation())
	    + restitution * initialVelocityProjection;

	float b = glm::dot(n1, n1 * body[0]->getInverseMass())
		+ glm::dot(w1, w1 * body[0]->getInvInersiaTensor())
		+ glm::dot(n2, n2 * body[1]->getInverseMass())
		+ glm::dot(w2, w2 * body[1]->getInvInersiaTensor());

	float lambda = (-a) / b;

	float velocityProjection =
		glm::dot(n1, (body[0]->getVelocity() + n1 * body[0]->getInverseMass() * lambda))
		+ glm::dot(w1, body[0]->getRotation() + w1 * body[0]->getInvInersiaTensor() * lambda)
		+ glm::dot(n2, (body[1]->getVelocity() + n2 * body[1]->getInverseMass() * lambda))
		+ glm::dot(w2, body[1]->getRotation() + w2 * body[1]->getInvInersiaTensor() * lambda);

	return lambda;
}



void Contact::aplly(const float &lambda)
{

	glm::vec3 velocityChange1 = body[0]->getVelocity() + contactNormal * lambda* body[0]->getInverseMass();
	glm::vec3 rotationChange1 = body[0]->getRotation() + glm::cross((contactNormal * lambda) , (contactPoint - body[0]->getPosition())) * body[0]->getInvInersiaTensor();

	body[0]->setVelocity(velocityChange1);
	body[0]->setRotation(rotationChange1);
	for(int i = 0; i < 2;i++)
		body[0]->setPosition(body[0]->getPosition() + contactNormal * penetration * 0.5f);

	if (body[1]->stat == true)
	{
		glm::vec3 velocityChange2 = body[1]->getVelocity() - contactNormal * lambda * body[1]->getInverseMass();
		glm::vec3 rotationChange2 = body[1]->getRotation() - glm::cross((contactNormal * lambda), (contactPoint - body[1]->getPosition())) * body[1]->getInvInersiaTensor();

		body[1]->setVelocity(velocityChange2);
		body[1]->setRotation(rotationChange2);
		for (int i = 0; i < 2; i++)
			body[1]->setPosition(body[1]->getPosition() - contactNormal * penetration * 0.5f);	
	}
}

float Contact::computeLambdaForOne()
{
	glm::vec3 n1 = glm::normalize(contactNormal);
	glm::vec3 w1 = glm::cross(contactNormal, (body[0]->getPosition() - contactPoint));

	float initialVelocityProjection =
		glm::dot(n1, body[0]->getVelocity())
		+ glm::dot(w1, body[0]->getRotation());

	float a = glm::dot(n1, body[0]->getVelocity())
		+ glm::dot(w1, body[0]->getRotation())
		+ restitution * initialVelocityProjection;

	float b = glm::dot(n1, n1 * body[0]->getInverseMass())
		+ glm::dot(w1, w1 * body[0]->getInvInersiaTensor());

	float lambda = (-a) / b;
	return lambda;
}

void Contact::applyForOne(const float& lambda)
{
	glm::vec3 velocityChange1 = body[0]->getVelocity() + contactNormal * lambda * body[0]->getInverseMass();
	glm::vec3 rotationChange1 = body[0]->getRotation() + glm::cross((contactNormal * lambda), (body[0]->getPosition() - contactPoint)) * body[0]->getInvInersiaTensor();

	body[0]->setVelocity(velocityChange1);
	body[0]->setRotation(rotationChange1);
	for (int i = 0; i < 1; i++)
		body[0]->setPosition(body[0]->getPosition() + contactNormal * penetration * 0.5f);
}

