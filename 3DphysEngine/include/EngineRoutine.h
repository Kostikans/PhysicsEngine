#pragma once

#include "Contact.h"
#include "AABB.h"
#include "Sphere.h"
#include "Plain.h"
#include "CollisionData.h"
#include "Gravity.h"

class EngineRoutine
{

	float accumulatedImpulse;
	float lambda;
	std::vector<Transformation*> objects;
	CollisionData* data;
	Gravity gravity;
	
public:
	EngineRoutine();
	void resolveContacts(Contact* contacts, float duration);
	void run(float deltaTime);
	void addEntity(Transformation* obj);
	void setData(CollisionData* data);
};