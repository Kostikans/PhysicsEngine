#include "..\include\EngineRoutine.h"
#include "..\include\CollisionDetector.h"


EngineRoutine::EngineRoutine()
{
	accumulatedImpulse = 0.0f;
	data = new CollisionData;		
}

void EngineRoutine::calculateProjects(Contact* contacts, int c)
{
	std::vector<float> projection;
	for (int i = 0; i < contacts->contactPoints.size(); ++i)
	{	
		glm::vec3 normal = glm::normalize(contacts->contactNormal);

		glm::vec3 n1 = normal;
		glm::vec3 w1 = (glm::cross(contacts->contactPoints[i] - contacts->body[0]->getPosition(), normal));
		glm::vec3 n2 = -normal;
		glm::vec3 w2 = -(glm::cross(contacts->contactPoints[i] - contacts->body[1]->getPosition(), normal));

		float initialVelocityProjection =
			glm::dot(n1, contacts->body[0]->getVelocity())
			+ glm::dot(w1, contacts->body[0]->getRotation())
			+ glm::dot(n2, contacts->body[1]->getVelocity())
			+ glm::dot(w2, contacts->body[1]->getRotation());


		projection.push_back(initialVelocityProjection);
	}
	proj.push_back(projection);
}

void EngineRoutine::resolveContacts(Contact* contacts, int c)
{
	glm::vec3 mostPerpendicular;
	float xProjection = fabs(glm::dot(contacts->contactNormal, glm::vec3(1.0f, 0.0f, 0.0f)));
	float yProjection = fabs(glm::dot(contacts->contactNormal, glm::vec3(0.0f, 1.0f, 0.0f)));
	float zProjection = fabs(glm::dot(contacts->contactNormal, glm::vec3(0.0f, 0.0f, 1.0f)));
	if ((xProjection > yProjection) && (xProjection > zProjection)) mostPerpendicular = glm::vec3(1.0f, 0.0f, 0.0f);
	else
		if ((yProjection > xProjection) && (yProjection > zProjection)) mostPerpendicular = glm::vec3(0.0f, 1.0f, 0.0f);
		else
			mostPerpendicular = glm::vec3(0.0f, 0.0f, 1.0f);

	glm::vec3 fDir1 = glm::normalize(glm::cross(contacts->contactNormal, mostPerpendicular));
	if (contacts->contactNormal == glm::vec3(glm::vec3(1.0f, 0.0f, 0.0f)))
		fDir1 = glm::vec3(0.0f, 1.0f, 0.0f);
	if (contacts->contactNormal == glm::vec3(glm::vec3(0.0f, 1.0f, 0.0f)))
		fDir1 = glm::vec3(1.0f, 0.0f, 0.0f);
	if (contacts->contactNormal == glm::vec3(glm::vec3(0.0f, 0.0f, 1.0f)))
		fDir1 = glm::vec3(0.0f, 1.0f, 0.0f);

	if (contacts->contactNormal == glm::vec3(glm::vec3(-1.0f, 0.0f, 0.0f)))
		fDir1 = glm::vec3(0.0f, 1.0f, 0.0f);
	if (contacts->contactNormal  == glm::vec3(glm::vec3(0.0f, -1.0f, 0.0f)))
		fDir1 = glm::vec3(1.0f, 0.0f, 0.0f);
	if (contacts->contactNormal == glm::vec3(glm::vec3(0.0f, 0.0f, -1.0f)))
		fDir1 = glm::vec3(0.0f, 1.0f, 0.0f);

	glm::vec3 fDir2 = glm::normalize(glm::cross(contacts->contactNormal, fDir1));

 	for (int i = 0; i < contacts->contactPoints.size(); ++i)
	{
		if (proj[c][i] > 0.0f)
			return;
		
		float lambda = contacts->computeLambda(i, proj[c][i], contacts->contactNormal);
		contacts->accumulatedImpulse += lambda;
		if (contacts->accumulatedImpulse < 0.0f)
		{
			lambda += (0.0f - contacts->accumulatedImpulse);
			contacts->accumulatedImpulse = 0.0f;
		}
		glm::vec3 impulse = lambda * contacts->contactNormal;
		contacts->applyLambda(impulse, i);
	
		

		float fricLambda1 = contacts->computeFrictionLambda(i, fDir1);
		float fricLambda2 = contacts->computeFrictionLambda(i, fDir2);
		
	
		glm::vec2 totalFric = glm::vec2(fricLambda1 , fricLambda2);

		if (glm::length(totalFric) > (contacts->friction * lambda) && fricLambda1 != 0.0f && fricLambda2 != 0.0f)
		{ 
			totalFric = glm::normalize(totalFric) * contacts->friction * lambda;
			fricLambda1 -= totalFric.x;
			fricLambda2 -= totalFric.y;
		}
		glm::vec3 fricImpulse1 = fDir1 * fricLambda1;
		glm::vec3 fricImpulse2 = fDir2 * fricLambda2;

		contacts->applyLambda(fricImpulse1, i);
		contacts->applyLambda(fricImpulse2, i);
	}
}

void EngineRoutine::run(float deltaTime,Shader& shader,Shader &debug)
{
 	for (int i = 0; i < objects.size(); ++i)
	{
		objects[i]->updateGravity(deltaTime);
	}
	for (int i = 0; i < objects.size() - 1; ++i)
	{
	    Transformation* current = objects[i];
		int FirstType = current->getType();
		for (int j = i + 1; j < objects.size(); ++j)
		{
		
			int SecondType = objects[j]->getType();
		
			if (FirstType == RIGIDBODY_TYPE_BOX)
			{
 				if (SecondType == RIGIDBODY_TYPE_BOX)
					CollisionDetector::boxVsBox(*dynamic_cast<AABB*>(current), *dynamic_cast<AABB*>(objects[j]), data);
				if (SecondType == RIGIDBODY_TYPE_SPHERE)
					CollisionDetector::boxAndSphere(*dynamic_cast<AABB*>(current), *dynamic_cast<Sphere*>(objects[j]), data);
				if (SecondType == RIGIDBODY_TYPE_PLANE)
					CollisionDetector::boxAndPlain(*dynamic_cast<AABB*>(current), *dynamic_cast<Plain*>(objects[j]), data);
			}
			if (FirstType == RIGIDBODY_TYPE_SPHERE)
			{
				if (SecondType == RIGIDBODY_TYPE_SPHERE)
					CollisionDetector::sphereAndSphere(*dynamic_cast<Sphere*>(current), *dynamic_cast<Sphere*>(objects[j]), data);
				if (SecondType == RIGIDBODY_TYPE_BOX)
					CollisionDetector::boxAndSphere(*dynamic_cast<AABB*>(objects[j]), *dynamic_cast<Sphere*>(current), data);
				if (SecondType == RIGIDBODY_TYPE_PLANE)
					CollisionDetector::sphereAndTruePlain(*dynamic_cast<Sphere*>(current), *dynamic_cast<Plain*>(objects[j]), data);
			}
			if (FirstType == RIGIDBODY_TYPE_PLANE)
			{
				if (SecondType == RIGIDBODY_TYPE_BOX)
					CollisionDetector::boxAndPlain(*dynamic_cast<AABB*>(objects[j]), *dynamic_cast<Plain*>(current), data);
				if (SecondType == RIGIDBODY_TYPE_SPHERE)
					CollisionDetector::sphereAndTruePlain(*dynamic_cast<Sphere*>(objects[j]), *dynamic_cast<Plain*>(current), data);
				if (SecondType == RIGIDBODY_TYPE_PLANE)
					continue;
			}
		}
	}
	/*for (int i = data->contactArray.size() - 1; i >= 0 ; --i)
	{
 		glm::vec3 localToGlobalA = glm::vec3(data->contactArray[i]->body[0]->getModelMatrix() * glm::vec4(data->contactArray[i]->localPosA, 1.0f));

		glm::vec3 localToGlobalB = glm::vec3(data->contactArray[i]->body[1]->getModelMatrix() * glm::vec4(data->contactArray[i]->localPosB, 1.0f));

		glm::vec3 rAB = localToGlobalA - localToGlobalB;

		glm::vec3 rA = data->contactArray[i]->globalPosA - localToGlobalA;

		glm::vec3 rB = data->contactArray[i]->globalPosB - localToGlobalB;

		bool stillPenetrating = glm::dot(rAB, data->contactArray[i]->contactNormal) <= 0.0f;

		bool rACloseEnough = glm::length(rA) < 0.01f;
		bool rBCloseEnough = glm::length(rB) < 0.01f;
	
		
		if (rACloseEnough && rBCloseEnough)
		{
			data->contactArray[i]->persistent = true;
		}
		else
		{
			std::cout<<data->contactArray[i]->accumulatedImpulse << std::endl;
			data->contactArray.erase(data->contactArray.begin() + i);
		}
	}
	int count = 0;
	if (data->contactArray.size() == 0 && data->lastContacts.size() != 0)
	{
		for (int j = 0; j < data->lastContacts.size(); ++j)
		{
			data->contactArray.push_back(data->lastContacts[j]);
		}
	}
	else
	{
		std::vector<Contact*> temp;
		for (int i = 0; i < data->lastContacts.size(); ++i)
		{
				for (int j = 0; j < data->contactArray.size(); ++j)
				{
					glm::vec3 rA = data->lastContacts[i]->globalPosA - data->contactArray[j]->globalPosA;
					glm::vec3 rB = data->lastContacts[i]->globalPosB - data->contactArray[j]->globalPosB;

					bool rAFarEnough = glm::length(rA) > 0.01f;
					bool rBFarEnough = glm::length(rB) > 0.01f;

					if ((rAFarEnough && rBFarEnough))
						++count;
				}
			if (count == data->contactArray.size())
				temp.push_back(data->lastContacts[i]);
			count = 0;
		}
		for (int i = 0; i < temp.size(); ++i)
		{
			data->contactArray.push_back(temp[i]);
		}
	}
*/
	data->contactPointView(debug);

	if (data->contactArray.empty() == 0)
	{
		for (int k = 0; k < data->contactArray.size(); ++k)
		{
			calculateProjects(data->contactArray[k], 0);
		}
		for (int i = 0; i < 15; ++i)
		{
			for (int j = 0; j < data->contactArray.size(); ++j)
			{
				resolveContacts(data->contactArray[j], j);
			}
		}
		//std::cout << data->contactArray[0]->body[0]->getVelocity().z << std::endl;
		proj.clear();
	}

	if (data->contactArray.empty() == 0)
	{
		for (int i = 0; i < data->contactArray.size(); ++i)
		{
			data->contactArray[i]->resolvePosition(i);
		}
	}

	data->contactArray.clear();

	for (int i = 0; i < objects.size(); ++i)
	{
		objects[i]->move(deltaTime);
	}
}

void EngineRoutine::addEntity(Transformation* obj)
{
	objects.push_back(obj);
}

void EngineRoutine::setData(CollisionData* data)
{
	this->data = data;
}
