#define _USE_MATH_DEFINES
#include "..\include\CollisionDetector.h"
#include <iostream>
#include <cmath>
#include <algorithm>


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
	contact->contactPoints.push_back(one.getPosition() + dist * 0.5f);
	contact->penetration = r - LengDist;
	contact->setBodyData(one.body, two.body, 0.0f, 0.8f);
	data->contactArray.push_back(std::move(contact));


	return true;
}

float CollisionDetector::penetrationOnAxis(const AABB& box1, const AABB& box2, const glm::vec3& axis, bool *isFlip)
{
	glm::vec3 VerBox1[8] =
	{
		glm::vec3(box1.halfSize().x  , box1.halfSize().y  , box1.halfSize().z),
		glm::vec3(-box1.halfSize().x  , box1.halfSize().y  ,  box1.halfSize().z),
		glm::vec3(box1.halfSize().x  ,  -box1.halfSize().y  , box1.halfSize().z),
		glm::vec3(box1.halfSize().x  ,  box1.halfSize().y  ,  -box1.halfSize().z),
		glm::vec3(-box1.halfSize().x  , -box1.halfSize().y  , -box1.halfSize().z),
		glm::vec3(box1.halfSize().x  , -box1.halfSize().y  ,  -box1.halfSize().z),
		glm::vec3(-box1.halfSize().x  ,  box1.halfSize().y  , -box1.halfSize().z),
		glm::vec3(-box1.halfSize().x  , -box1.halfSize().y  ,  box1.halfSize().z)
	};
	glm::vec3 VerBox2[8] =
	{
		glm::vec3(box2.halfSize().x  , box2.halfSize().y  , box2.halfSize().z),
		glm::vec3(-box2.halfSize().x  , box2.halfSize().y  ,  box2.halfSize().z),
		glm::vec3(box2.halfSize().x  ,  -box2.halfSize().y  , box2.halfSize().z),
		glm::vec3(box2.halfSize().x  ,  box2.halfSize().y  ,  -box2.halfSize().z),
		glm::vec3(-box2.halfSize().x  , -box2.halfSize().y  , -box2.halfSize().z),
		glm::vec3(box2.halfSize().x  , -box2.halfSize().y  ,  -box2.halfSize().z),
		glm::vec3(-box2.halfSize().x  ,  box2.halfSize().y  , -box2.halfSize().z),
		glm::vec3(-box2.halfSize().x  ,  -box2.halfSize().y  ,  box2.halfSize().z)
	};
	
	for (int i = 0; i < 8; ++i)
	{
		VerBox1[i] = glm::vec3(box1.body->getModelMatrix() * glm::vec4(VerBox1[i], 1.0f));
		VerBox2[i] = glm::vec3(box2.body->getModelMatrix() * glm::vec4(VerBox2[i], 1.0f));
	}

	float resultMin1;
	float resultMax1;
	float resultMin2;
	float resultMax2;
	resultMin1 = resultMax1 = glm::dot(axis, VerBox1[0]);
	resultMin2 = resultMax2 = glm::dot(axis, VerBox2[0]);

	for (int i = 1; i < 8; ++i) {
		float projection = glm::dot(axis, VerBox1[i]);
		resultMin1 = (projection < resultMin1) ? projection : resultMin1;
		resultMax1 = (projection > resultMax1) ? projection : resultMax1;
	}

	for (int i = 1; i < 8; ++i) {
		float projection = glm::dot(axis, VerBox2[i]);
		resultMin2 = (projection < resultMin2) ? projection : resultMin2;
		resultMax2 = (projection > resultMax2) ? projection : resultMax2;
	}

	if (!((resultMin2 <= resultMax1) && (resultMin1 <= resultMax2))) {
		return 0.0f; 
	}

	float len1 = resultMax1 - resultMin1;
	float len2 = resultMax2 - resultMin2;
	float min = fminf(resultMin1, resultMin2);
	float max = fmaxf(resultMax1, resultMax2);
	float length = max - min;

	if (isFlip != 0) {
		*isFlip = (resultMin2 < resultMin1);
	}

	return (len1 + len2) - length;
}

bool CollisionDetector::boxVsBox(const AABB& box1, const AABB& box2, CollisionData* data)
{
	glm::vec3 VerBox1[8] =
	{
		glm::vec3(box1.halfSize().x  , box1.halfSize().y  , box1.halfSize().z),
		glm::vec3(-box1.halfSize().x  , box1.halfSize().y  ,  box1.halfSize().z),
		glm::vec3(box1.halfSize().x  ,  -box1.halfSize().y  , box1.halfSize().z),
		glm::vec3(box1.halfSize().x  ,  box1.halfSize().y  ,  -box1.halfSize().z),
		glm::vec3(-box1.halfSize().x  , -box1.halfSize().y  , -box1.halfSize().z),
		glm::vec3(box1.halfSize().x  , -box1.halfSize().y  ,  -box1.halfSize().z),
		glm::vec3(-box1.halfSize().x  ,  box1.halfSize().y  , -box1.halfSize().z),
		glm::vec3(-box1.halfSize().x  , - box1.halfSize().y  ,  box1.halfSize().z)
	};
	glm::vec3 VerBox2[8] =
	{
		glm::vec3(box2.halfSize().x  , box2.halfSize().y  , box2.halfSize().z),
		glm::vec3(-box2.halfSize().x  , box2.halfSize().y  ,  box2.halfSize().z),
		glm::vec3(box2.halfSize().x  ,  -box2.halfSize().y  , box2.halfSize().z),
		glm::vec3(box2.halfSize().x  ,  box2.halfSize().y  ,  -box2.halfSize().z),
		glm::vec3(-box2.halfSize().x  , -box2.halfSize().y  , -box2.halfSize().z),
		glm::vec3(box2.halfSize().x  , -box2.halfSize().y  ,  -box2.halfSize().z),
		glm::vec3(-box2.halfSize().x  ,  box2.halfSize().y  , -box2.halfSize().z),
		glm::vec3(-box2.halfSize().x  ,  -box2.halfSize().y  ,  box2.halfSize().z)
	};
	for (int i = 0; i < 8; ++i)
	{
		VerBox1[i] = glm::vec3(box1.body->getModelMatrix() * glm::vec4(VerBox1[i], 1.0f));
		VerBox2[i] = glm::vec3(box2.body->getModelMatrix() * glm::vec4(VerBox2[i], 1.0f));
	}
	glm::vec3 toCenter = box2.getPosition() - box1.getPosition();

	std::vector<glm::vec3> normals;
	normals.push_back(box1.getNormX());
	normals.push_back(box1.getNormY());
	normals.push_back(box1.getNormZ());
	normals.push_back(box2.getNormX());
	normals.push_back(box2.getNormY());
	normals.push_back(box2.getNormZ());

	if ((normals[0] != normals[3]))
	{
		normals.push_back(glm::cross(normals[0], normals[3]));
		normals.push_back(glm::cross(normals[0], normals[4]));
		normals.push_back(glm::cross(normals[0], normals[5]));
	}
	if ((normals[1] != normals[4]))
	{
		normals.push_back(glm::cross(normals[1], normals[3]));
		normals.push_back(glm::cross(normals[1], normals[4]));
		normals.push_back(glm::cross(normals[1], normals[5]));
	}
	if ((normals[2] != normals[5]))
	{
		normals.push_back(glm::cross(normals[2], normals[3]));
		normals.push_back(glm::cross(normals[2], normals[4]));
		normals.push_back(glm::cross(normals[2], normals[5]));
	}
	

	bool isflip;
	float penetration = 1000.0f;
	glm::vec3 hitNormal;
	for (int i = 0; i < normals.size(); ++i)
	{
		float depth = penetrationOnAxis(box1, box2, normals[i], &isflip);
		if (depth <= 0.0f)
			return false;
		else if (depth < penetration) {
			if (isflip) {
				//normals[i] = normals[i] * -1.0f;
			}
			penetration = depth;
			hitNormal = normals[i];
		}
	}
	if (hitNormal == glm::vec3(0.0f))
		return false;

	glm::vec3 axis = glm::normalize(hitNormal);
	std::vector<Line> edges1;
	std::vector<Line> edges2;

	/*int index[][2] = {
		{1,6},{3,6},{4,6},{7,2},{2,5},{2,0},
		{1,0},{0,3},{7,1},{7,4},{4,5},{3,5}
	};*/
	int index[][2] = { 
		{ 6, 1 },{ 6, 3 },{ 6, 4 },{ 2, 7 },{ 2, 5 },{ 2, 0 },
		{ 0, 1 },{ 0, 3 },{ 7, 1 },{ 7, 4 },{ 4, 5 },{ 5, 3 }
	};

	for (int j = 0; j < 12; ++j) 
	{
		edges1.push_back(Line(VerBox1[index[j][0]] , VerBox1[index[j][1]]));
		edges2.push_back(Line(VerBox2[index[j][0]] , VerBox2[index[j][1]]));
	}

	std::vector<Plane> plane1;

	plane1.push_back(Plane(normals[0]           , glm::dot(normals[0], box1.getPosition() + normals[0] * box1.halfSize().x)));
	plane1.push_back(Plane(normals[0]  * -1.0f  , -glm::dot(normals[0], box1.getPosition() - normals[0] * box1.halfSize().x)));
	plane1.push_back(Plane(normals[1]           , glm::dot(normals[1], box1.getPosition() + normals[1] * box1.halfSize().y)));
	plane1.push_back(Plane(normals[1]  * -1.0f  , -glm::dot(normals[1], box1.getPosition() - normals[1] * box1.halfSize().y)));
	plane1.push_back(Plane(normals[2]           , glm::dot(normals[2], box1.getPosition() + normals[2] * box1.halfSize().z)));
	plane1.push_back(Plane(normals[2]  * -1.0f  , -glm::dot(normals[2], box1.getPosition() - normals[2] * box1.halfSize().z)));
									      																				

	std::vector<Plane> plane2;

	plane2.push_back(Plane(normals[3]           , glm::dot(normals[3], box2.getPosition() + normals[3] * box2.halfSize().x)));
	plane2.push_back(Plane(normals[3]  * -1.0f  , -glm::dot(normals[3], box2.getPosition() - normals[3] * box2.halfSize().x)));
	plane2.push_back(Plane(normals[4]           , glm::dot(normals[4], box2.getPosition() + normals[4] * box2.halfSize().y)));
	plane2.push_back(Plane(normals[4]  * -1.0f  , -glm::dot(normals[4], box2.getPosition() - normals[4] * box2.halfSize().y)));
	plane2.push_back(Plane(normals[5]           , glm::dot(normals[5], box2.getPosition() + normals[5] * box2.halfSize().z)));
	plane2.push_back(Plane(normals[5]  * -1.0f  , -glm::dot(normals[5], box2.getPosition() - normals[5] * box2.halfSize().z)));

	std::vector<glm::vec3> points2 = clipEdges(edges1, plane2, box2);
	std::vector<glm::vec3> points1 = clipEdges(edges2, plane1, box1);


	float resultMin1;
	float resultMax1;
	resultMin1 = resultMax1 = glm::dot(axis, VerBox1[0]);
	for (int i = 1; i < 8; ++i) {
		float projection = glm::dot(axis, VerBox1[i]);
		resultMin1 = (projection < resultMin1) ? projection : resultMin1;
		resultMax1 = (projection > resultMax1) ? projection : resultMax1;
	}
	float distance = (resultMax1 - resultMin1) * 0.5f - penetration * 0.5f;
	glm::vec3 pointOnPlane = box1.getPosition() + axis * distance;

	Contact* contact = new Contact;
	contact->contactNormal = axis;
	contact->penetration = penetration ;

	for (int i = 0; i < points1.size(); ++i)
		contact->contactPoints.push_back(points1[i]  + (axis * glm::dot(axis, pointOnPlane - points1[i])));
	for (int i = 0; i < points2.size(); ++i)
		contact->contactPoints.push_back(points2[i] +  (axis * glm::dot(axis, pointOnPlane - points2[i])));

	for (int i = contact->contactPoints.size() - 1; i >= 0; --i)
	{
		for (int j = contact->contactPoints.size() - 1; j > i; --j)
		{
			if (glm::length(contact->contactPoints[i] - contact->contactPoints[j]) < 0.0001f) 
			{
				contact->contactPoints.erase(contact->contactPoints.begin() + j);
				break;
			}
		}
	}

	contact->setBodyData(box2.body, box1.body, 0.0f, 0.2f);
	data->contactArray.push_back(std::move(contact));
	return true;
}

std::vector<glm::vec3> CollisionDetector::clipEdges(const std::vector<Line>& edges, const std::vector<Plane> & planes, const AABB& box)
{
	std::vector<glm::vec3> result;

	glm::vec3 intersection;
	for (int i = 0; i < planes.size(); ++i) {
		for (int j = 0; j < edges.size(); ++j) {
			if (clipPlane(edges[j], planes[i], &intersection)) {
				if (PointInOBB(intersection, box)) {
					result.push_back(intersection);
				}
			}
		}
	}

	return result;
}

bool CollisionDetector::PointInOBB(const glm::vec3& point, const AABB& obb)
{
	glm::vec3 dir = point - obb.getPosition();
	glm::vec3 normals[3];
	normals[0] = obb.getNormX();
	normals[1] = obb.getNormY();
	normals[2] = obb.getNormZ();
	float half[3];
	half[0] = obb.halfSize().x;
	half[1] = obb.halfSize().y;
	half[2] = obb.halfSize().z;
	for (int i = 0; i < 3; ++i) {
	
		float distance = glm::dot(dir, normals[i]);

		if (distance > half[i]) {
			return false;
		}
		if (distance < -half[i]) {
			return false;
		}
	}

	return true;
}

bool CollisionDetector::clipPlane(const Line &edge, const Plane &plane, glm::vec3* intersectionPoint)
{
	glm::vec3 ab = edge.end - edge.start;

	float nA = glm::dot(plane.normal, edge.start);
	float nAB = glm::dot(plane.normal, ab);

	
	float t = (plane.offset - nA) / nAB;
	if (t >= 0.0f && t <= 1.0f) {
		if (intersectionPoint != nullptr)
		{
			*intersectionPoint = edge.start + ab * t;
		}
		return true;
	}

	return false;
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
	contact->contactPoints.push_back(position - plane.getDirection() * centerDistance);
	contact->setBodyData(sph2.body, plane.body, 0.0f, 1.0f);
	data->contactArray.push_back(std::move(contact));

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
	contact->contactPoints.push_back(closestPtWorld);
	contact->setBodyData(sphere.body, aabb.body, 0.0f, 0.7f);
	data->contactArray.push_back(std::move(contact));

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
	
	float offset = glm::dot(plane.getDirection(), plane.getPosition());

	bool collide = false;
	float pLen = aabb.halfSize().x * fabsf(glm::dot(plane.getDirection(), aabb.getNormX())) +
		aabb.halfSize().y * fabsf(glm::dot(plane.getDirection(), aabb.getNormY())) +
		aabb.halfSize().z * fabsf(glm::dot(plane.getDirection(), aabb.getNormZ()));

	float dist = glm::dot(plane.getDirection(), aabb.getPosition()) - offset;

	if (fabsf(dist) <= pLen)
	{

		Contact* contact = new Contact;
		contact->contactNormal = plane.getDirection();
		for (int i = 0; i < 8; ++i)
		{
			glm::vec3 VertexPos = Vertexes[i];
			float vertexDistance = glm::dot(VertexPos, plane.getDirection());
			if (vertexDistance < offset)
			{
				if (glm::length(VertexPos - plane.getPosition()) > plane.getWidth() ||
					glm::length(VertexPos - plane.getPosition()) > plane.getHeight())
					return false;
				
	
				contact->contactPoints.push_back(plane.getDirection() * (offset - vertexDistance) + VertexPos);
				contact->penetration = offset - vertexDistance;
				
			}
		}
		contact->contactNormal = plane.getDirection();

		contact->setBodyData(aabb.body, plane.body, 0.0f, 0.1f);
		data->contactArray.push_back(std::move(contact));
	}

	return collide;


}

