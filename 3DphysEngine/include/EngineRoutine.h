#pragma once

#include "Contact.h"

class EngineRoutine
{

	float accumulatedImpulse;
	float lambda;
	float LinearProjectionPercent; 
	float PenetrationSlack;
public:
	EngineRoutine();
	void resolveContacts(Contact* contacts, unsigned numContacts, float duration);
};