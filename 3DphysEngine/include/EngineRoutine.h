#pragma once

#include "Contact.h"

class EngineRoutine
{
	unsigned int velocityIterations;
	unsigned int positionIterations;

	unsigned velocityIterationsUsed;
	unsigned positionIterationsUsed;

	float velocityEpsilon;
	float positionEpsilon;

	float accumulatedImpulse;
	float lambda;

	void prepareContacts(Contact* contactArray, unsigned numContacts,float duration);
	void adjustVelocities(Contact* c, unsigned numContacts, float duration);
	void adjustPositions(Contact* c, unsigned numContacts, float duration);
public:
	EngineRoutine();
	void resolveContacts(Contact* contacts, unsigned numContacts, float duration);
};