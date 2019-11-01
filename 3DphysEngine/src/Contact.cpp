#include "..\include\Contact.h"
#include <iostream>

float Contact::LinearProjectionPercent = 0.45f;
float Contact::PenetrationSlack = 0.01f;
void Contact::setBodyData(RigidBody* one, RigidBody* two, float friction, float restitution)
{
	Contact::body[0] = one;
	Contact::body[1] = two;
	Contact::friction = friction;
	Contact::restitution = restitution;
}


float Contact::computeLambda(int c, float proj)
{
	glm::vec3 normal = glm::normalize(contactNormal);
	glm::vec3 n1 = normal;
	glm::vec3 w1 = (glm::cross(contactPoints[c] - body[0]->getPosition(), normal));
	glm::vec3 n2 = -normal;
	glm::vec3 w2 = -(glm::cross(contactPoints[c] - body[1]->getPosition(), normal));

	float a = glm::dot(n1, body[0]->getVelocity())
		+ glm::dot(n2, body[1]->getVelocity())
		+ glm::dot(w1, body[0]->getRotation())
		+ glm::dot(w2, body[1]->getRotation()) + restitution * proj;

	float b = glm::dot(n1 , n1 * body[0]->getInverseMass())
		+ glm::dot(w1 , w1 * body[0]->getInvInersiaTensor())
		+ glm::dot(n2 , n2 * body[1]->getInverseMass())
		+ glm::dot(w2 , w2 * body[1]->getInvInersiaTensor());

   	float lambda = -a / b;  

	//float velocityProjection =
	//	  glm::dot(n1 , (body[0]->getVelocity() + n1 * body[0]->getInverseMass() * lambda))
	//	+ glm::dot(w1 , (body[0]->getRotation() + w1 * body[0]->getInvInersiaTensor() * lambda))
	//	+ glm::dot(n2 , (body[1]->getVelocity() + n2 * body[1]->getInverseMass() * lambda))
	//	+ glm::dot(w2 , (body[1]->getRotation() + w2 * body[1]->getInvInersiaTensor() * lambda));


	//std::cout << "velProsj: " <<velocityProjection << "  -rest * proj:" << -restitution * proj << std::endl;
	//std::cout << " lambda:  " << lambda << std::endl;


 	return lambda;
}

void Contact::applyLambda(float lambda, int c)
{
  	glm::vec3 impulse = glm::normalize(contactNormal) * lambda;
   	body[0]->setVelocity(body[0]->getVelocity() + impulse * body[0]->getInverseMass());
	body[1]->setVelocity(body[1]->getVelocity() - impulse * body[1]->getInverseMass());

	//std::cout << "body rot before:  " << body[0]->getRotation().x << " " << body[0]->getRotation().y << " " << body[0]->getRotation().z;
    body[0]->addRotation((glm::cross(contactPoints[c] - body[0]->getPosition(), impulse)) * body[0]->getInvInersiaTensor());
	//std::cout << "   body rot after:  " << body[0]->getRotation().x << " " << body[0]->getRotation().y << " " << body[0]->getRotation().z << std::endl;
	body[1]->addRotation(-(glm::cross(contactPoints[c] - body[1]->getPosition(), impulse)) * body[1]->getInvInersiaTensor());
}

float Contact::computePseudoLambda(int c, float proj)
{
	glm::vec3 normal = contactNormal;

	glm::vec3 n1 = normal;
	glm::vec3 w1 = (glm::cross(contactPoints[c] - body[0]->getPosition(), normal));
	glm::vec3 n2 = -normal;
	glm::vec3 w2 = -(glm::cross(contactPoints[c] - body[1]->getPosition(), normal));

	float a = glm::dot(n1, body[0]->getVelocity())
		+ glm::dot(n2, body[1]->getVelocity())
		+ glm::dot(w1, body[0]->getRotation())
		+ glm::dot(w2, body[1]->getRotation()) + penetration * 0.8f ;

	float b = glm::dot(n1, n1 * body[0]->getInverseMass())
		+ glm::dot(w1, w1 * body[0]->getInvInersiaTensor())
		+ glm::dot(n2, n2 * body[1]->getInverseMass())
		+ glm::dot(w2, w2 * body[1]->getInvInersiaTensor());

	float lambda = -a / b;
	return lambda;
}

void Contact::applyPseudoVelocities(float lambda, int c)
{
	glm::vec3 impulse = glm::normalize(contactNormal) * lambda;
	body[0]->setPseudoVelocity(impulse * body[0]->getInverseMass());
	body[1]->setPseudoVelocity(-impulse * body[1]->getInverseMass());

	body[0]->setPseudoRotation((glm::cross(contactPoints[c] - body[0]->getPosition(), impulse)) * body[0]->getInvInersiaTensor());
	body[1]->setPseudoRotation(-(glm::cross(contactPoints[c] - body[1]->getPosition(), impulse)) * body[1]->getInvInersiaTensor());
}

void Contact::resolvePosition(int c)
{
	/*float linearInertia[2];
	float angularInertia[2];

	float linearMove[2];
	float angularMove[2];
	float totalInertia = 0.0f;

	for (int i = 0; i < 2; ++i)
	{
		glm::mat3x3 invTensor = body[i]->getInvInersiaTensor();

		glm::vec3 angularInertiaWorld = glm::cross(contactPoints[c] - body[i]->getPosition(), contactNormal) * body[i]->getInvInersiaTensor();
		angularInertia[i] = glm::dot(angularInertiaWorld , contactNormal);

		linearInertia[i] = body[i]->getInverseMass();

		totalInertia += linearInertia[i] + angularInertia[i];
	}
	float inverseInertia = 1.0f / totalInertia;
	linearMove[0] = penetration * linearInertia[0] * inverseInertia / contactPoints.size();
	linearMove[1] = -penetration * linearInertia[1] * inverseInertia / contactPoints.size();
	angularMove[0] = penetration * angularInertia[0] * inverseInertia / contactPoints.size();
	angularMove[1] = -penetration * angularInertia[1] * inverseInertia / contactPoints.size();

	body[0]->setPosition(body[0]->getPosition() + contactNormal * linearMove[0]);

	if (!body[1]->getInverseMass() == 0.0f)
	{
		body[1]->setPosition(body[1]->getPosition() - contactNormal * linearMove[1]);
	}
	for (int i = 0; i < 2; ++i)
	{
		if (angularInertia[i] == 0.0f)
			return;
		glm::vec3 angularChange;
		angularChange = glm::cross(contactPoints[c] - body[i]->getPosition(), contactNormal) * body[i]->getInvInersiaTensor() 
			* (angularMove[i] / angularInertia[i]);

		glm::quat q = body[i]->getOrientation();

		glm::quat temp;
		temp.w = 0.0f;
		temp.x = angularChange.x;
		temp.y = angularChange.y;
		temp.z = angularChange.z;
		temp *= q;
		temp *= 0.5f;
		q += temp;
		body[i]->setOrientation(q);
	}*/
	for (int i = 0; i < 1; ++i)
	{
		float depth = fmaxf(penetration - PenetrationSlack, 0.0f);
		float totalMass = body[0]->getInverseMass() + body[1]->getInverseMass();
		float scalar = depth / totalMass;
		glm::vec3 correction = contactNormal * scalar * LinearProjectionPercent;
		
		body[0]->setPosition(body[0]->getPosition() + correction * body[0]->getInverseMass());
	
		if (!body[1]->getInverseMass() == 0.0f)
		{
			body[1]->setPosition(body[1]->getPosition() - correction * body[1]->getInverseMass());
		}
	}
}

