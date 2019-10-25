#include "..\include\EngineRoutine.h"


EngineRoutine::EngineRoutine()
{
	accumulatedImpulse = 0.0f;

	lambda =0.0f;
}

void EngineRoutine::resolveContacts(Contact* contacts, unsigned numContacts, float duration)
{
	/*for (int i = 0; i < contacts->contactPoints.size(); ++i)
	{
		glm::vec3 r1 = contacts->contactPoints[i] - contacts->body[0]->getPosition();
		glm::vec3 r2 = contacts->contactPoints[i] - contacts->body[1]->getPosition();

		glm::vec3 relativeVel = (contacts->body[0]->getVelocity() + glm::cross(contacts->body[0]->getRotation(), r1)) - 
			(contacts->body[1]->getVelocity() + glm::cross(contacts->body[1]->getRotation(), r2));
		glm::vec3 relativeNorm = glm::normalize(contacts->contactNormal);

		if (glm::dot(relativeVel, relativeNorm) > 0.0f)
		{
			return;
		}
	}*/
	std::vector<float> projects;
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
		projects.push_back(initialVelocityProjection);
	}
	if (contacts->body[1]->getInverseMass() != 0.0f)
	{
		int kek = 0;
	}
 	 accumulatedImpulse = 0.0f;
	 for (int j = 0; j < 30; ++j)
	 {
		 
			for (int i = 0; i < contacts->contactPoints.size(); ++i)
			{
				
			  //   lambda = contacts->computeLambda(i);
				 ///*accumulatedImpulse += lambda;
			  //   if (accumulatedImpulse < 0.0f)
			  //   {
			  //      lambda += (0.0f - accumulatedImpulse);
			  //      accumulatedImpulse = 0.0f;
			  //   }*/
				 //if (lambda < 0)
					// continue;
			  //   contacts->aplly(lambda, i);
			  //   //contacts->apllyImpulses(i);
				float lambda = contacts->kekCompute(i, projects[i]);
				if (lambda < 0)
					continue;
				contacts->kekApply(lambda, i);
				 
			}	 
	 }
	
     contacts->resolvePosition();
	contacts->contactPoints.clear();
}
