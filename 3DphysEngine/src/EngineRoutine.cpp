#include "..\include\EngineRoutine.h"

void EngineRoutine::prepareContacts(Contact* contactArray, unsigned numContacts, float duration)
{
	
	//Contact* lastContact = contactArray + numContacts;
	//for (Contact* contact = contactArray; contact < lastContact; contact++)
	//{
		contactArray->calculateInternals(duration);
	//}
	velocityIterations = 2;
	positionIterations = 2;
}

void EngineRoutine::adjustVelocities(Contact* c, unsigned numContacts, float duration)
{
	glm::vec3 velocityChange[2], rotationChange[2];
	glm::vec3 deltaVel;

	velocityIterationsUsed = 0;
	while (velocityIterationsUsed < velocityIterations)
	{
		float max = 0.01f;
		unsigned index = numContacts;
		for (unsigned i = 0; i < numContacts; i++)
		{
			if (c[i].desiredDeltaVelocity > max)
			{
				max = c[i].desiredDeltaVelocity;
				index = i;
			}
		}
		if (index == numContacts) break;
		c[index].applyVelocityChange(velocityChange, rotationChange);

	/*	for (unsigned i = 0; i < numContacts; i++)
		{
			for (unsigned b = 0; b < 2; b++) if (c[i].body[b])
			{
				for (unsigned d = 0; d < 2; d++)
				{
					if (c[i].body[b] == c[index].body[d])
					{
						deltaVel = velocityChange[d] + glm::cross(rotationChange[d], c[i].relativeContactPosition[b]);
						c[i].contactVelocity += c[i].contactToWorld * deltaVel * (float)(b ? -1 : 1);
						c[i].calculateDesiredDeltaVelocity(duration);
					}
				}
			}
		}*/
		velocityIterationsUsed++;
	}
}

void EngineRoutine::adjustPositions(Contact* c, unsigned numContacts, float duration)
{
	unsigned i, index;
	glm::vec3 linearChange[2], angularChange[2];
	float max;
    glm::vec3 deltaPosition;

	positionIterationsUsed = 0;
	while (positionIterationsUsed < positionIterations)
	{
		max = 0.01f;
		index = numContacts;
		for (i = 0; i < numContacts; i++)
		{
			if (c[i].penetration > max)
			{
				max = c[i].penetration;
				index = i;
			}
		}
		if (index == numContacts) break;

		c[index].applyPositionChange(linearChange,angularChange,max);

		/*for (i = 0; i < numContacts; i++)
		{
			for (unsigned b = 0; b < 2; b++) if (c[i].body[b])
			{
				for (unsigned d = 0; d < 2; d++)
				{
					if (c[i].body[b] == c[index].body[d])
					{
						deltaPosition = linearChange[d] + glm::cross(angularChange[d] , c[i].relativeContactPosition[b]);
						c[i].penetration +=	glm::dot(deltaPosition,c[i].contactNormal) * (float)(b ? 1 : -1);
					}
				}
			}
		}*/
		positionIterationsUsed++;
	}
}

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
		accumulatedImpulse += lambda;
		if (accumulatedImpulse < 0.0f)
		{
			lambda += (0.0f - accumulatedImpulse);
			accumulatedImpulse = 0.0f;
		}
		contacts->applyForOne(lambda);
	}
	
}
