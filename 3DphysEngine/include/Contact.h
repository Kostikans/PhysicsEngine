#pragma once

#include "Transformation.h"
#include "RigidBody.h"



class Contact
{
public:
	RigidBody* body[2];

	float friction;
	float restitution;
	glm::vec3 contactPoint;
	glm::vec3 contactNormal;
	float penetration;
 

	void setBodyData(RigidBody* one, RigidBody* two, float friction, float restitution);

	glm::vec3 calculateFrictionlessImpulse(glm::mat3x3 *inverseInertiaTensor);
	glm::vec3 calculateFrictionImpulse(glm::mat3x3 *inverseInertiaTensor);



	float computeLambda();
	void aplly(const float& lambda);

	float computeLambdaForOne();
	void applyForOne(const float& lambda);
};