#include "..\include\EngineRoutine.h"


EngineRoutine::EngineRoutine()
{
	accumulatedImpulse = 0.0f;
	lambda =0.0f;
}

void EngineRoutine::resolveContacts(Contact* contacts, unsigned numContacts, float duration)
{


	glm::vec3 relativeVelocity = contacts->body[0]->getVelocity();
	relativeVelocity = contacts->body[0]->getVelocity() - contacts->body[1]->getVelocity();
	 if (glm::dot(relativeVelocity, contacts->contactNormal) > 0 && contacts->body[1]->stat != false)
		return;
	// else
	// {
	//	 relativeVelocity = contacts->body[0]->getVelocity();
	//	 if (glm::dot(relativeVelocity, contacts->contactNormal) > 0)
	//		 return;
	//}

    lambda = 0.0f;
	if (contacts->body[1]->stat == true)
	{
		lambda = contacts->computeLambda();
		if (lambda < 0)
			return;
		accumulatedImpulse += lambda;
		if (accumulatedImpulse < 0.0f)
		{
			lambda += (0.0f - accumulatedImpulse);
			accumulatedImpulse = 0.0f;
		}
		contacts->aplly(lambda);
	}
	else
	{
		lambda = contacts->computeLambdaForOne();
		if (lambda < 0)
			return;
		accumulatedImpulse += lambda;
		if (accumulatedImpulse < 0.0f)
		{
			lambda += (0.0f - accumulatedImpulse);
			accumulatedImpulse = 0.0f;
		}
		contacts->applyForOne(lambda);
	}
	
}
