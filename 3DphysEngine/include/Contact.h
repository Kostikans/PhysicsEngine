#pragma once

#include "Transformation.h"
#include "RigidBody.h"



class Contact
{
public:
	RigidBody* body[2];

	float friction;
	float restitution;
	glm::vec3 contactNormal;
	float penetration;
	std::vector <glm::vec3> contactPoints;

	void setBodyData(RigidBody* one, RigidBody* two, float friction, float restitution);

	
	static float LinearProjectionPercent;
	static float PenetrationSlack;


	float computeLambda(int c);
	void aplly(const float lambda, int c);

	void apllyImpulses(int c);

	void resolvePosition();
};