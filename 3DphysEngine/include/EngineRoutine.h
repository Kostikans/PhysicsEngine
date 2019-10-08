#pragma once

#include "Contact.h"

class EngineRoutine
{

	float accumulatedImpulse;
	float lambda;

public:
	EngineRoutine();
	void resolveContacts(Contact* contacts, unsigned numContacts, float duration);
};