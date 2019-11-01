#include "..\include\EngineRoutine.h"
#include "..\include\CollisionDetector.h"


EngineRoutine::EngineRoutine()
{
	accumulatedImpulse = 0.0f;
	data = new CollisionData;		
}

void EngineRoutine::calculateProjects(Contact* contacts, int c)
{
	std::vector<float> tempProj;
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

		tempProj.push_back(initialVelocityProjection);
	}
	proj.push_back(tempProj);
}

void EngineRoutine::resolveContacts(Contact* contacts, int c)
{
	for (int i = 0; i < contacts->contactPoints.size(); ++i)
	{
		if (proj[c][i] > 0.0f)
			return;

		float lambda = contacts->computeLambda(i, proj[c][i]);
		accumulatedImpulse += lambda;
		//float pseudo = contacts->computePseudoLambda(0, proj[c]);
		if (accumulatedImpulse < 0.0f)
		{
			lambda += (0.0f - accumulatedImpulse);
			accumulatedImpulse = 0.0f;
		}

		contacts->applyLambda(lambda, i);
	}
	//contacts->applyPseudoVelocities(pseudo, 0);	
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
	data->contactPointView(debug);
	
	if (data->contactArray.empty() == 0)
	{
		for (int k = 0; k < data->contactArray.size(); ++k)
		{
			calculateProjects(data->contactArray[k], k);
		}
		for (int i = 0; i < 15; ++i)
		{
			for (int j = 0; j < data->contactArray.size(); ++j)
			{
				resolveContacts(data->contactArray[j], j);
			}		
		}
		accumulatedImpulse = 0.0f;
	}

	if (data->contactArray.empty() == 0)
	{
		for (int i = 0; i < data->contactArray.size(); ++i)
		{
			data->contactArray[i]->resolvePosition(i);
		}
	}
	
	proj.clear();
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
