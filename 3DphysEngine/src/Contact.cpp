#include "..\include\Contact.h"
#include <iostream>

float Contact::LinearProjectionPercent = 0.5f;
float Contact::PenetrationSlack = 0.01f;
void Contact::setBodyData(RigidBody* one, RigidBody* two, float friction, float restitution)
{
	Contact::body[0] = one;
	Contact::body[1] = two;
	Contact::friction = friction;
	Contact::restitution = restitution;
}

float Contact::computeLambda(int c)
{

	float invMass1 = body[0]->getInverseMass();
	float invMass2 = body[1]->getInverseMass();
	float invMassSum = invMass1 + invMass2;

	
	glm::vec3 r1 = contactPoints[c] - body[0]->getPosition();
	glm::vec3 r2 = contactPoints[c] - body[1]->getPosition();
	glm::mat3x3 i1 = body[0]->getInvInersiaTensor();
	glm::mat3x3 i2 = body[1]->getInvInersiaTensor();

	glm::vec3 relativeVel = (body[0]->getVelocity() + glm::cross(body[0]->getRotation(), r1)) - (body[1]->getVelocity() + glm::cross(body[1]->getRotation(), r2));
	glm::vec3 relativeNorm = contactNormal;
	relativeNorm = glm::normalize(relativeNorm);

	if (glm::dot(relativeVel, relativeNorm) > 0.0f)
		return 0.0f;

	float e = restitution;

	float numerator = (-(1.0f + e) * glm::dot(relativeVel, relativeNorm));
	float d1 = invMassSum;

	glm::vec3 d2 = glm::cross(glm::cross(r1, relativeNorm) * i1, r1);
	glm::vec3 d3 = glm::cross(glm::cross(r2, relativeNorm) * i2, r2);
	float denominator = d1 + glm::dot(relativeNorm, d2 + d3);

	float j = (denominator == 0.0f) ? 0.0f : numerator / denominator;
	if (contactPoints.size() > 0.0f && j != 0.0f) {
		j /= (float)contactPoints.size();
	}
	return j;
}



void Contact::aplly(float lambda , int c)
{

	float invMass1 = body[0]->getInverseMass();
	float invMass2 = body[1]->getInverseMass();
	float invMassSum = invMass1 + invMass2;
	glm::mat3x3 i1 = body[0]->getInvInersiaTensor();
	glm::mat3x3 i2 = body[1]->getInvInersiaTensor();
	glm::vec3 r1 = contactPoints[c] - body[0]->getPosition();
	glm::vec3 r2 = contactPoints[c] - body[1]->getPosition();
	glm::vec3 relativeNorm = glm::normalize(contactNormal);

	glm::vec3 impulse = relativeNorm * lambda;

	body[0]->setVelocity(body[0]->getVelocity() + impulse * invMass1);
	body[1]->setVelocity(body[1]->getVelocity() - impulse * invMass2);

	body[0]->setRotation(body[0]->getRotation() + (glm::cross(r1, impulse) * i1));
	body[1]->setRotation(body[1]->getRotation() - (glm::cross(r2, impulse) * i2));
}


void Contact::apllyImpulses(int c)
{
	float invMass1 = body[0]->getInverseMass();
	float invMass2 = body[1]->getInverseMass();
	float invMassSum = invMass1 + invMass2;

	if (invMassSum == 0.0f) {
		return;
	}
	glm::vec3 r1 = contactPoints[c] - body[0]->getPosition();
	glm::vec3 r2 = contactPoints[c] - body[1]->getPosition();
	glm::mat3x3 i1 = body[0]->getInvInersiaTensor();
	glm::mat3x3 i2 = body[1]->getInvInersiaTensor();

	glm::vec3 relativeVel = (body[0]->getVelocity() + glm::cross(body[0]->getRotation(), r1)) - (body[1]->getVelocity() + glm::cross(body[1]->getRotation(), r2));
	glm::vec3 relativeNorm = contactNormal;
	relativeNorm = glm::normalize(relativeNorm);
	if (body[1]->getInverseMass() != 0.0f)
	{
 		std::cout << "lel";
	}

	if (glm::dot(relativeVel, relativeNorm) > 0.0f) {
		return;
	}
	float e = restitution;

	float numerator = (-(1.0f + e) * glm::dot(relativeVel, relativeNorm));
	float d1 = invMassSum;

  	glm::vec3 d2 = glm::cross(glm::cross(r1, relativeNorm) * i1, r1);
	glm::vec3 d3 = glm::cross(glm::cross(r2, relativeNorm) * i2, r2);
	float denominator = d1 + glm::dot(relativeNorm, d2 + d3);

   	float j = (denominator == 0.0f) ? 0.0f : numerator / denominator;
	if (contactPoints.size() > 0.0f && j != 0.0f) {
	      j /= (float)contactPoints.size();
	}

	if (j < 0)
		return;
 	glm::vec3 impulse = relativeNorm * j ;

	body[0]->setVelocity(body[0]->getVelocity() - impulse * invMass1);
	body[1]->setVelocity(body[1]->getVelocity() - impulse * invMass2);

	body[0]->setRotation(body[0]->getRotation() + (glm::cross(r1, impulse) * i1));
	body[1]->setRotation(body[1]->getRotation() - (glm::cross(r2, impulse) * i2));
}

float Contact::kekCompute(int c, float proj)
{
	
	glm::vec3 normal = contactNormal;

	glm::vec3 n1 = normal;
	glm::vec3 w1 = (glm::cross(contactPoints[c] - body[0]->getPosition(), normal));
	glm::vec3 n2 = -normal;
	glm::vec3 w2 = -(glm::cross(contactPoints[c] - body[1]->getPosition(), normal));

	//float initialVelocityProjection =
	//	  glm::dot(n1 , body[0]->getVelocity())
	//	+ glm::dot(w1 , body[0]->getRotation())
	//	+ glm::dot(n2 , body[1]->getVelocity())
	//	+ glm::dot(w2 , body[1]->getRotation());

	if (proj > 0.0f)
		return 0.0f;

	float a = glm::dot(n1 , body[0]->getVelocity())
		+ glm::dot(n2 , body[1]->getVelocity())
		+ glm::dot(w1 , body[0]->getRotation())
		+ glm::dot(w2 , body[1]->getRotation()) + restitution * proj;

	float b = glm::dot(n1 , n1 * body[0]->getInverseMass())
		+ glm::dot(w1 , w1 * body[0]->getInvInersiaTensor())
		+ glm::dot(n2 , n2 * body[1]->getInverseMass())
		+ glm::dot(w2 , w2 * body[1]->getInvInersiaTensor());


   	float lambda = -a / b;  
	float velocityProjection =
		  glm::dot(n1 , (body[0]->getVelocity() + n1 * body[0]->getInverseMass() * lambda))
		+ glm::dot(w1 , (body[0]->getRotation() + w1 * body[0]->getInvInersiaTensor() * lambda))
		+ glm::dot(n2 , (body[1]->getVelocity() + n2 * body[1]->getInverseMass() * lambda))
		+ glm::dot(w2 , (body[1]->getRotation() + w2 * body[1]->getInvInersiaTensor() * lambda));


	lambda /= contactPoints.size();
 	return lambda;
}

void Contact::kekApply(float lambda, int c)
{

   	glm::vec3 impulse = glm::normalize(contactNormal) * lambda;
	body[0]->setVelocity(body[0]->getVelocity() + impulse * body[0]->getInverseMass());
	body[1]->setVelocity(body[1]->getVelocity() - impulse * body[1]->getInverseMass());

	body[0]->setRotation(body[0]->getRotation() + (glm::cross(contactPoints[c] - body[0]->getPosition(), impulse)) * body[0]->getInvInersiaTensor());
	body[1]->setRotation(body[1]->getRotation() - (glm::cross(contactPoints[c] - body[1]->getPosition(), impulse)) * body[1]->getInvInersiaTensor());
}

void Contact::resolvePosition()
{
	for (int i = 0; i < 1; ++i)
	{
		float depth = fmaxf(penetration - PenetrationSlack, 0.0f);
		float totalMass = body[0]->getInverseMass() + body[1]->getInverseMass();
		float scalar = depth ;
		glm::vec3 correction = contactNormal * scalar * LinearProjectionPercent;
		
		body[0]->setPosition(body[0]->getPosition() + correction);
	
		if (!body[1]->getInverseMass() == 0.0f)
		{
			body[1]->setPosition(body[1]->getPosition() - correction);
		}
	}
}

