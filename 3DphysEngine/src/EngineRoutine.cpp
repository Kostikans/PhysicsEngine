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


	glm::vec3 relativeVelocity = contacts->body[0]->getVelocity();
	relativeVelocity = contacts->body[0]->getVelocity() - contacts->body[1]->getVelocity() ;
	 if (glm::dot(relativeVelocity, contacts->contactNormal) > 0 && contacts->body[1]->stat == true)
		return;
	 /*else
	 {
		 relativeVelocity = contacts->body[0]->getVelocity();
		 if (glm::dot(relativeVelocity, contacts->contactNormal) > 0)
			 return;
	}*/

	 glm::vec3 vel1 = contacts->body[0]->getVelocity();
	 glm::vec3 rot1 = contacts->body[0]->getRotation();
	 glm::vec3 vel2 = contacts->body[1]->getVelocity();
	 glm::vec3 rot2 = contacts->body[1]->getRotation();
	 for (int j = 0; j < 1; ++j)
	 {	
	
		 accumulatedImpulse = 0.0f;
		 for (int i = 0; i < contacts->contactPoints.size(); ++i)
		 {
			
			 if (contacts->body[1]->stat == true)
			 {
				 lambda = contacts->computeLambda(i,vel1,rot1,vel2,rot2);
				 accumulatedImpulse += lambda;
				if (accumulatedImpulse < 0.0f)
				{
					 lambda += (0.0f - accumulatedImpulse);
					 accumulatedImpulse = 0.0f;
				 }
				 contacts->aplly(lambda, i);
			 }
			 else
			 {			
				 lambda = contacts->computeLambdaForOne(i,vel1,rot1);	
				 accumulatedImpulse += lambda;
				 if (accumulatedImpulse < 0.0f)
				 {
					lambda += (0.0f - accumulatedImpulse);
				    accumulatedImpulse = 0.0f;
				 }
				 contacts->applyForOne(lambda, i);
			 }
		 }
		 //contacts->body[0]->update(duration);
		 //contacts->body[1]->update(duration);
	 }

	if (contacts->body[1]->stat == false)
		contacts->resolvePositionForOne();

	else
		contacts->resolvePosition();

	contacts->contactPoints.clear();
}
