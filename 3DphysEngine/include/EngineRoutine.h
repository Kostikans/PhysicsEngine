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
	std::vector<Transformation*> objects;
	CollisionData* data;
	Gravity gravity;
	std::vector<std::vector<float>> proj;
	
public:
	EngineRoutine();
	void calculateProjects(Contact* contacts , int c);
	void resolveContacts(Contact* contacts, int c);
	void run(float deltaTime, Shader & shader, Shader & debug);
	void addEntity(Transformation* obj);
	void setData(CollisionData* data);
};