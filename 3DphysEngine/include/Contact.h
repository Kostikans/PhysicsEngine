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

	glm::mat3x3 contactToWorld;
	glm::vec3 contactVelocity;

	float desiredDeltaVelocity;

	glm::vec3 relativeContactPosition[2];

	void setBodyData(RigidBody* one, RigidBody* two, float friction, float restitution);
	void calculateInternals(float duration);
	void swapBodies();
	void calculateDesiredDeltaVelocity(float duration);
	glm::vec3 calculateLocalVelocity(unsigned bodyIndex,float duration);	
	void calculateContactBasis();	
	void applyImpulse(const glm::vec3 &impulse, RigidBody *body, glm::vec3 *velocityChange, glm::vec3 *rotationChange);
	void applyVelocityChange(glm::vec3 velocityChange[2], glm::vec3 rotationChange[2]);	
	void applyPositionChange(glm::vec3 linearChange[2], glm::vec3 angularChange[2], float penetration);
	glm::vec3 calculateFrictionlessImpulse(glm::mat3x3 *inverseInertiaTensor);
	glm::vec3 calculateFrictionImpulse(glm::mat3x3 *inverseInertiaTensor);



	float computeLambda();
	void aplly(const float& lambda);


	void apppppply();

	float computeLambdaForOne();
	void applyForOne(const float& lambda);
};