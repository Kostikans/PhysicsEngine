#pragma once

#include "Contact.h"

class CollisionData
{
public:
	std::vector<Contact*> contactArray;

	int contactsLeft;
	unsigned contactCount;
	float friction;
	float restitution;
	float tolerance;

	CollisionData() {};
};