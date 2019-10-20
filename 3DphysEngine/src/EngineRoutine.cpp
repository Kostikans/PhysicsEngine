#include "..\include\EngineRoutine.h"


EngineRoutine::EngineRoutine()
{
	accumulatedImpulse = 0.0f;
	LinearProjectionPercent = 0.4f;
	PenetrationSlack = 0.04f;
	lambda =0.0f;
}

void EngineRoutine::resolveContacts(Contact* contacts, unsigned numContacts, float duration)
{
	/*for (int i = 0; i < contacts->contactPoints.size(); ++i)
	{
		glm::vec3 r1 = contacts->contactPoints[i] - contacts->body[0]->getPosition();
		glm::vec3 r2 = contacts->contactPoints[i] - contacts->body[1]->getPosition();

		glm::vec3 relativeVel = (contacts->body[1]->getVelocity() + glm::cross(contacts->body[1]->getRotation(), r2)) - 
			(contacts->body[0]->getVelocity() + glm::cross(contacts->body[0]->getRotation(), r1));
		glm::vec3 relativeNorm = glm::normalize(relativeNorm);

		if (glm::dot(relativeVel, relativeNorm) > 0.0f)
		{
			return;
		}
	}*/
	 accumulatedImpulse = 0.0f;
	 for (int j = 0; j < 15; ++j)
	 {
		 
			for (int i = 0; i < contacts->contactPoints.size(); ++i)
			{
				
			    /* lambda = contacts->computeLambda(i);
				 accumulatedImpulse += lambda;
			     if (accumulatedImpulse < 0.0f)
			     {
			        lambda += (0.0f - accumulatedImpulse);
			        accumulatedImpulse = 0.0f;
			     }*/
				 if (lambda < 0)
					 continue;
			     contacts->aplly(lambda, i);
			     //contacts->apllyImpulses(i);
				 
			}	 
	 }
    contacts->resolvePosition();
	contacts->contactPoints.clear();
}
