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


	float computeLambda(int c,glm::vec3 v1, glm::vec3 r1, glm::vec3 v2, glm::vec3 r2);
	void aplly(const float& lambda, int c);

	float computeLambdaForOne(int c , glm::vec3 v , glm::vec3 r);
	void applyForOne(const float& lambda , int c);

	void resolvePosition();
	void resolvePositionForOne();
};