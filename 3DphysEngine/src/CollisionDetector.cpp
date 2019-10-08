#include "..\include\CollisionDetector.h"
#include <iostream>
#define SGN(x) sgn(x)

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
	glm::vec3 VerBox1[8] =
	{
		glm::vec3(-box1.halfSize().x  , -box1.halfSize().y  , -box1.halfSize().z),
		glm::vec3(-box1.halfSize().x  , -box1.halfSize().y  ,  box1.halfSize().z),
		glm::vec3(-box1.halfSize().x  ,  box1.halfSize().y  , -box1.halfSize().z),
		glm::vec3(-box1.halfSize().x  ,  box1.halfSize().y  ,  box1.halfSize().z),
		glm::vec3(box1.halfSize().x  , -box1.halfSize().y  , -box1.halfSize().z),
		glm::vec3(box1.halfSize().x  , -box1.halfSize().y  ,  box1.halfSize().z),
		glm::vec3(box1.halfSize().x  ,  box1.halfSize().y  , -box1.halfSize().z),
		glm::vec3(box1.halfSize().x  ,  box1.halfSize().y  ,  box1.halfSize().z)
	};
	glm::vec3 VerBox2[8] =
	{
		glm::vec3(-box2.halfSize().x  , -box2.halfSize().y  , -box2.halfSize().z),
		glm::vec3(-box2.halfSize().x  , -box2.halfSize().y  ,  box2.halfSize().z),
		glm::vec3(-box2.halfSize().x  ,  box2.halfSize().y  , -box2.halfSize().z),
		glm::vec3(-box2.halfSize().x  ,  box2.halfSize().y  ,  box2.halfSize().z),
		glm::vec3(box2.halfSize().x  , -box2.halfSize().y  , -box2.halfSize().z),
		glm::vec3(box2.halfSize().x  , -box2.halfSize().y  ,  box2.halfSize().z),
		glm::vec3(box2.halfSize().x  ,  box2.halfSize().y  , -box2.halfSize().z),
		glm::vec3(box2.halfSize().x  ,  box2.halfSize().y  ,  box2.halfSize().z)
	};
	for (int i = 0; i < 8; ++i)
	{
		VerBox1[i] = glm::vec3(box1.body->getModelMatrix() * glm::vec4(VerBox1[i], 1.0f));
		VerBox2[i] = glm::vec3(box2.body->getModelMatrix() * glm::vec4(VerBox2[i], 1.0f));
	}
	glm::vec3 toCenter = box2.getPosition() - box1.getPosition();

	glm::vec3 n1 = box1.getNormX();
	glm::vec3 n2 = box1.getNormY();
	glm::vec3 n3 = box1.getNormZ();
					
	glm::vec3 n4 = box2.getNormX();
	glm::vec3 n5 = box2.getNormY();
	glm::vec3 n6 = box2.getNormZ();

	glm::vec3 n7  = glm::cross(n1,n4);
	glm::vec3 n8  = glm::cross(n1,n5);
	glm::vec3 n9  = glm::cross(n1,n6);
					
	glm::vec3 n10 = glm::cross(n2,n4);
	glm::vec3 n11 = glm::cross(n2,n5);
	glm::vec3 n12 = glm::cross(n2,n6);
					
	glm::vec3 n13 = glm::cross(n3,n4);
	glm::vec3 n14 = glm::cross(n3,n5);
	glm::vec3 n15 = glm::cross(n3,n6);

	float Rxx = glm::dot(n1, n4);
	float Rxy = glm::dot(n1, n5);
	float Rxz = glm::dot(n1, n6);
	float Ryx = glm::dot(n5, n1);
	float Ryy = glm::dot(n2, n5);
	float Ryz = glm::dot(n2, n6);
	float Rzz = glm::dot(n3, n6);
	float Rzy = glm::dot(n3, n5);
	float Rzx = glm::dot(n3, n4);

	float offset1;
	float offset2;
	float offset3;
	float offset4;
	float offset5;
	float offset6;
	float offset7;
	float offset8; 
	float offset9;
	float offset10;
	float offset11;
	float offset12;
	float offset13;
	float offset14;
	float offset15;

	offset1 = box1.halfSize().x + fabs(box2.halfSize().x * Rxx)
		+ fabs(box2.halfSize().y * Rxy) + fabs(box2.halfSize().z * Rxz);
	if (fabs(glm::dot(toCenter, n1)) > offset1)
			return false;

	offset2 = box1.halfSize().y + fabs(box2.halfSize().x * Ryx)
		+ fabs(box2.halfSize().y * Ryy) + fabs(box2.halfSize().z * Ryz);
	if (fabs(glm::dot(toCenter, n2)) > offset2)
			return false;

	offset3 = box1.halfSize().z + fabs(box2.halfSize().x * Rzx)
		+ fabs(box2.halfSize().y * Rzy) + fabs(box2.halfSize().z * Rzz);
	if (fabs(glm::dot(toCenter, n3)) > offset3)
			return false;

	offset4 = box2.halfSize().x + fabs(box1.halfSize().x * Rxx)
		+ fabs(box1.halfSize().y * Ryx) + fabs(box1.halfSize().z * Rzx);
	if (fabs(glm::dot(toCenter, n4)) > offset4)
			return false;

	offset5 = box2.halfSize().y + fabs(box1.halfSize().x * Rxy)
		+ fabs(box1.halfSize().y * Ryy) + fabs(box1.halfSize().z * Rzy);
	if (fabs(glm::dot(toCenter, n5)) > offset5)
			return false;

	offset6 = box2.halfSize().z + fabs(box1.halfSize().x * Rxz)
		+ fabs(box1.halfSize().y * Ryz) + fabs(box1.halfSize().z * Rzz);
	if (fabs(glm::dot(toCenter, n6)) > offset6)
			return false;

	offset7 = box1.halfSize().y * Rzx
		+ fabs(box1.halfSize().z * Ryx)
		+ fabs(box2.halfSize().y * Rxz)
		+ fabs(box2.halfSize().z * Rxy);
	if (fabs(glm::dot(toCenter, glm::cross(n1,n4))) > offset7)
			return false;

	offset8 = box1.halfSize().y * Rzy
		+ fabs(box1.halfSize().z * Ryy)
		+ fabs(box2.halfSize().x * Rxy)
		+ fabs(box2.halfSize().z * Rxx);
	if (fabs(glm::dot(toCenter, glm::cross(n1, n5))) > offset8)
		return false;

	offset9 = box1.halfSize().y * Rzz
		+ fabs(box1.halfSize().z * Ryz)
		+ fabs(box2.halfSize().x * Rxy)
		+ fabs(box2.halfSize().y * Rxx);
	if (fabs(glm::dot(toCenter, glm::cross(n1, n6))) > offset9)
		return false;

	offset10 = box1.halfSize().x * Rzx
		+ fabs(box1.halfSize().z * Rxx)
		+ fabs(box2.halfSize().y * Ryz)
		+ fabs(box2.halfSize().z * Ryy);
	if (fabs(glm::dot(toCenter, glm::cross(n2, n4))) > offset10)
		return false;

	offset11 = box1.halfSize().x * Rzy
		+ fabs(box1.halfSize().z * Rxy)
		+ fabs(box2.halfSize().x * Ryz)
		+ fabs(box2.halfSize().z * Ryx);
	if (fabs(glm::dot(toCenter, glm::cross(n2, n5))) > offset11)
		return false;

	offset12 = box1.halfSize().x * Rzz
		+ fabs(box1.halfSize().z * Rxz)
		+ fabs(box2.halfSize().x * Ryy)
		+ fabs(box2.halfSize().y * Ryx);
	if (fabs(glm::dot(toCenter, glm::cross(n2, n6))) > offset12)
		return false;

	offset13 = box1.halfSize().x * Ryx
		+ fabs(box1.halfSize().y* Rxx)
		+ fabs(box2.halfSize().y* Rzz)
		+ fabs(box2.halfSize().z * Rzy);
	if (fabs(glm::dot(toCenter, glm::cross(n3, n4))) > offset13)
		return false;

	offset14 = box1.halfSize().x * Ryy
		+ fabs(box1.halfSize().y * Rxy)
		+ fabs(box2.halfSize().x * Rzz)
		+ fabs(box2.halfSize().z * Rzx);
	if (fabs(glm::dot(toCenter, glm::cross(n3, n5))) > offset14)
		return false;

	offset15 = box1.halfSize().x * Ryz
		+ fabs(box1.halfSize().y* Rxz)
		+ fabs(box2.halfSize().x* Rzy)
		+ fabs(box2.halfSize().z* Rzx);
	if (fabs(glm::dot(toCenter, glm::cross(n3, n6))) > offset15)
		return false;
	float max;
	
	return true;
	
}

bool CollisionDetector::overLapOnAxis(const AABB& box1, const AABB& box2, const glm::vec3 axis)
{
	float oneProject = transformToAxis(box1, axis);
	float twoProject = transformToAxis(box2, axis);

	glm::vec3 toCenter = box2.getPosition() - box1.getPosition();
	float distance = fabs(glm::dot(toCenter , axis));
	return (distance < oneProject + twoProject);
}

float CollisionDetector::transformToAxis(const AABB& box, const glm::vec3 axis)
{
	return  fabs(glm::dot(axis , (box.getNormX() * box.halfSize().x))) +
			fabs(glm::dot(axis , (box.getNormY() * box.halfSize().y))) +
			fabs(glm::dot(axis , (box.getNormZ() * box.halfSize().z)));
}

float CollisionDetector::sgn(float& x)
{
	if (x > 0)
		return (std::move(x));
	else
		if (x == 0.0f)
			return 0.0f;
		else
			return (std::move(-x));
}

bool CollisionDetector::sphereAndTruePlane(const Sphere& sph2,const Plain& plane , CollisionData* data)
{

	glm::vec3 position = sph2.getPosition(); 
    float centerDistance = glm::dot(plane.getDirection() , position) -  glm::dot(plane.getPosition() , plane.getDirection());
	if (centerDistance * centerDistance  > sph2.getRadius() * sph2.getRadius())
	{
		return false;
	}
	float t = glm::dot(plane.getDirection(), position) - glm::dot(plane.getPosition(), plane.getDirection());
	glm::vec3 point = position - t *  plane.getDirection();

	if (glm::length(point - plane.getPosition()) > plane.getWidth() ||
		glm::length(point - plane.getPosition()) > plane.getHeight())
		return false;
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
			if (glm::length(VertexPos - plane.getPosition()) > plane.getWidth() ||
				glm::length(VertexPos - plane.getPosition()) > plane.getHeight())
				return false;
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

