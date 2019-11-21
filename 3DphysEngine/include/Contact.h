#pragma once

#include "Transformation.h"
#include "RigidBody.h"



class Contact
{
public:
	RigidBody* body[2];

	float accumulatedImpulse;
	float friction;
	float restitution;
	glm::vec3 contactNormal;
	float penetration;
	std::vector <glm::vec3> contactPoints;
	glm::vec3 localPosA;
	glm::vec3 localPosB;
	glm::vec3 globalPosA;
	glm::vec3 globalPosB;
	unsigned persistent;

	Contact();

	void setBodyData(RigidBody* one, RigidBody* two, float friction, float restitution);

	static float LinearProjectionPercent;
	static float PenetrationSlack;

	float computeLambda(int c, float proj, const glm::vec3& normal);
	void applyLambda(glm::vec3& impulse, int c);

	float computePseudoLambda(int c, float proj);
	float computeFrictionLambda(int c, const glm::vec3& normal);
	void applyPseudoVelocities(float lambda, int c);
	void resolvePosition(int c);
};