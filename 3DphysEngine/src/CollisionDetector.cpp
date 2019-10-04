#include "..\include\CollisionDetector.h"

bool CollisionDetector::sphereAndSphere(const Sphere& one, const Sphere& two, CollisionData* data)
{
	float r = one.getRadius() + two.getRadius();
	glm::vec3 dist = one.getPosition() - two.getPosition();
	float LengDist = glm::length(dist);
	if (LengDist - r > 0 || LengDist == 0.0f)
	{
		return false;
	}
	glm::vec3 normal = dist * (1.0f / LengDist);

	Contact* contact = new Contact;
	contact->contactNormal = normal;
	contact->contactPoint = one.getPosition() + dist * 0.5f;
	contact->penetration = r - LengDist;
	contact->setBodyData(one.body, two.body, 0.0f, 0.8f);
	data->contactArray.push_back(std::move(contact));
	data->addContacts(1);

	return true;
}

bool CollisionDetector::boxAndBox(const AABB& box1, const AABB& box2, CollisionData* data)
{
	return false;
}

bool CollisionDetector::sphereAndTruePlane(const Sphere& sph2,const Plain& plane , CollisionData* data)
{

	glm::vec3 position = sph2.getPosition(); 
    float centerDistance = glm::length(plane.getDirection() * position -  plane.getPosition() * plane.getDirection());
	if (centerDistance  > sph2.getRadius())
	{
		return false;
	}
	glm::vec3 normal = plane.getDirection();
	float penetration = -centerDistance;
	if (centerDistance < 0)
	{
		normal *= -1;
		penetration = -penetration;
	}
	penetration += sph2.getRadius();

	Contact* contact = new Contact;
	contact->contactNormal = plane.getDirection();
	contact->penetration = penetration;
	contact->contactPoint = position - plane.getDirection() * centerDistance;
	contact->setBodyData(sph2.body, plane.body, 0.0f, 1.0f);
	data->contactArray.push_back(std::move(contact));
	data->addContacts(1);
	return true;
}

bool CollisionDetector::boxAndSphere(const AABB& aabb, const Sphere& sphere, CollisionData* data)
{
	
	glm::vec3 center = sphere.getPosition();


	glm::vec3 relCenter = glm::vec3(glm::inverse(aabb.body->getModelMatrix()) * glm::vec4(center, 1.0f));

	if ((fabs(relCenter.x) - sphere.getRadius() - aabb.halfSize().x  > 0) ||
		(fabs(relCenter.y) - sphere.getRadius() - aabb.halfSize().y  > 0) ||
		(fabs(relCenter.z) - sphere.getRadius() - aabb.halfSize().z  > 0))
	{
		return 0;
	}
	glm::vec3 closestPt = glm::vec3(0.0f);
	float dist = 0.0f;

	dist = relCenter.x;
	if (dist > aabb.halfSize().x) dist = aabb.halfSize().x ;
	if (dist < -aabb.halfSize().x) dist = -aabb.halfSize().x ;
	closestPt.x = dist;
	dist = relCenter.y;
	if (dist > aabb.halfSize().y) dist = aabb.halfSize().y ;
	if (dist < -aabb.halfSize().y) dist = -aabb.halfSize().y ;
	closestPt.y = dist;
	dist = relCenter.z;
	if (dist > aabb.halfSize().z) dist = aabb.halfSize().z ;
	if (dist < -aabb.halfSize().z) dist = -aabb.halfSize().z;
	closestPt.z = dist;

	dist = glm::length(closestPt - relCenter);
	if (dist > sphere.getRadius() * sphere.getRadius()) return 0;

	glm::vec3 closestPtWorld = glm::vec3(aabb.body->getModelMatrix() * glm::vec4(closestPt, 1.0f));

	Contact* contact = new Contact;
	contact->contactNormal = glm::normalize(center - closestPtWorld);
	contact->penetration = sphere.getRadius() - sqrtf(dist);
	contact->contactPoint = closestPtWorld;
	contact->setBodyData(sphere.body, aabb.body, 0.0f, 0.6f);
	data->contactArray.push_back(std::move(contact));
	data->addContacts(1);
	return true; 
}

bool CollisionDetector::boxAndPlain(const AABB& aabb, const Plain& plane, CollisionData* data)
{
	glm::vec3 Vertexes[8] =
	{
		glm::vec3(-aabb.halfSize().x  , -aabb.halfSize().y  , -aabb.halfSize().z ),
		glm::vec3(-aabb.halfSize().x  , -aabb.halfSize().y  , aabb.halfSize().z ),
		glm::vec3(-aabb.halfSize().x  , aabb.halfSize().y  ,- aabb.halfSize().z ),
		glm::vec3(-aabb.halfSize().x  , aabb.halfSize().y  , aabb.halfSize().z ),
		glm::vec3( aabb.halfSize().x  , -aabb.halfSize().y  , -aabb.halfSize().z ),
		glm::vec3( aabb.halfSize().x  , -aabb.halfSize().y  , aabb.halfSize().z ),
		glm::vec3( aabb.halfSize().x  , aabb.halfSize().y  ,- aabb.halfSize().z ),
		glm::vec3( aabb.halfSize().x  , aabb.halfSize().y  , aabb.halfSize().z )
	};
	for (unsigned i = 0; i < 8; i++)
	{
		Vertexes[i] = glm::vec3(aabb.body->getModelMatrix() * glm::vec4(Vertexes[i], 1.0f));
	}
	bool collide = false;
	for (int i = 0; i < 8; ++i)
	{
		glm::vec3 VertexPos = Vertexes[i];
		float vertexDistance = glm::dot(VertexPos, plane.getDirection());
		float offset = glm::dot(plane.getDirection(), plane.getPosition());
		if (vertexDistance < offset)
		{
			collide = true;
			Contact* contact = new Contact;
			contact->contactNormal = plane.getDirection();
			contact->penetration = offset - vertexDistance;
			contact->contactPoint = plane.getDirection() * (offset - vertexDistance) + VertexPos;

			contact->setBodyData(aabb.body, plane.body, 0.0f, 0.5f);
			data->contactArray.push_back(std::move(contact));
			data->addContacts(1);
		}
	}
	return collide;


}

