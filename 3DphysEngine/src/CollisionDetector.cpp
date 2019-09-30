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

bool CollisionDetector::sphereAndTruePlane(const Sphere& sph2,const Plain& plane , CollisionData* data)
{

	glm::vec3 position = sph2.getPosition(); 
    float centerDistance = glm::length(plane.getDirection() * position -  plane.getPosition() * plane.getDirection());
	if (centerDistance * centerDistance > sph2.getRadius() * sph2.getRadius())
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
	contact->contactNormal = normal;
	contact->penetration = penetration;
	contact->contactPoint = position - plane.getDirection() * centerDistance;
	contact->setBodyData(sph2.body, plane.body, 0.0f, 0.8f);
	data->contactArray.push_back(std::move(contact));
	data->addContacts(1);
	return true;
}

bool CollisionDetector::boxAndSphere(const AABB& aabb, const Sphere& sphere, CollisionData* data)
{
	glm::vec3 sphPos = sphere.getPosition();
	glm::vec3 closestPoint = Geometry3D::ClosestPoint(aabb, sphere.getPosition());
	float distanceSq = glm::length(closestPoint - sphere.getPosition());
	if (distanceSq > sphere.getRadius())
	{
		return false;
	}
	glm::vec3 normal = glm::normalize(closestPoint - aabb.getPosition());
	glm::vec3 outsidePoint = sphPos - normal * sphere.getRadius();
	float distance = glm::length(closestPoint - outsidePoint);

	Contact* contact = new Contact;
	contact->contactNormal = normal;
	contact->penetration = distance * 0.5f;
	contact->contactPoint = (closestPoint + (outsidePoint - closestPoint) * 0.5f);
	contact->setBodyData(sphere.body, aabb.body, 0.0f, 0.5f);
	data->contactArray.push_back(std::move(contact));
	data->addContacts(1);
	return true;
}

bool CollisionDetector::boxAndPlain(const AABB& aabb, const Plain& plane, CollisionData* data)
{

	float width = aabb.getWidth();
	float height = aabb.getHeight();
	float depth = aabb.getDepth();

	glm::vec3 Vertexes[8] = {
		aabb.getMin(),aabb.getMin() + glm::vec3(width,0.0f,0.0f),
		aabb.getMin() + glm::vec3(0.0f,0.0f,-depth) ,aabb.getMin() + glm::vec3(width,0.0f,-depth),

		aabb.getMax(),aabb.getMax() + glm::vec3(-width,0.0f,0.0f),

		aabb.getMax() + glm::vec3(0.0f,0.0f,depth),aabb.getMax() + glm::vec3(width,0.0f,depth)
	};
	bool collide = false;
	for (int i = 0; i < 8; ++i)
	{
		glm::vec3 VertexPos = Vertexes[i];
		float vertexDistance = glm::dot(VertexPos, plane.getDirection());
		float offset = glm::dot(plane.getDirection(), plane.getPosition());
		if (vertexDistance <= offset)
		{
			collide = true;
			Contact* contact = new Contact;
			contact->contactNormal = plane.getDirection();
			contact->penetration = offset - vertexDistance;
			contact->contactPoint = VertexPos;
			contact->setBodyData(aabb.body, plane.body, 0.0f, 0.4f);

			data->contactArray.push_back(std::move(contact));
			data->addContacts(1);
		}
	}
	return collide;
}

