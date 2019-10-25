#define _USE_MATH_DEFINES
#include "..\include\CollisionDetector.h"
#include <iostream>
#include <cmath>
#include <algorithm>

float CollisionDetector::penetrationOnAxis(const AABB& box1, const AABB& box2, const glm::vec3& axis, const glm::vec3& toCentre)
{
	float oneProject = transformToAxis(box1, axis);
	float twoProject = transformToAxis(box2, axis);

	float distance = fabs(glm::dot(toCentre , axis));
	return oneProject + twoProject - distance;
}

float CollisionDetector::transformToAxis(const AABB& box, const glm::vec3& axis)
{
	return
		box.halfSize().x * fabs(glm::dot(axis , box.getNormX())) +
		box.halfSize().y * fabs(glm::dot(axis , box.getNormY())) +
		box.halfSize().z * fabs(glm::dot(axis , box.getNormZ()));
}

bool CollisionDetector::tryAxis(const AABB& box1, const AABB& box2,  glm::vec3& axis, const glm::vec3& toCentre, unsigned index, float& smallestPenetration, unsigned& smallestCase)
{
	if (glm::length(axis) < 0.0001f) return true;
	axis = glm::normalize(axis);

	float penetration = penetrationOnAxis(box1, box2, axis, toCentre);

	if (penetration < 0.0f) return false;
	if (penetration < smallestPenetration) {
		smallestPenetration = penetration;
		smallestCase = index;
	}
	return true;
}

bool CollisionDetector::boxAndBox(const AABB& box1, const AABB& box2, CollisionData* data)
{
	glm::vec3 toCentre = box2.getPosition() - box1.getPosition();

	float pen = 1000.0f;
	unsigned best = 160;

	glm::vec3 n1 = box1.getNormX();
	glm::vec3 n2 = box1.getNormY();
	glm::vec3 n3 = box1.getNormZ();
	glm::vec3 n4 = box2.getNormX();
	glm::vec3 n5 = box2.getNormY();
	glm::vec3 n6 = box2.getNormZ();

	glm::vec3 n7 = glm::cross(n1, n3);
	glm::vec3 n8 = glm::cross(n1, n4);
	glm::vec3 n9 = glm::cross(n1, n5);
	glm::vec3 n10 = glm::cross(n2, n3);
	glm::vec3 n11 = glm::cross(n2, n4);
	glm::vec3 n12 = glm::cross(n2, n5);
	glm::vec3 n13 = glm::cross(n3, n3);
	glm::vec3 n14 = glm::cross(n3, n4);
	glm::vec3 n15 = glm::cross(n3, n5);


	if (!tryAxis(box1, box2, n1 , toCentre, 0, pen, best))  return false;
	if (!tryAxis(box1, box2, n2, toCentre, 1, pen, best))   return false;
	if (!tryAxis(box1, box2, n3, toCentre, 2, pen, best))   return false;
	if (!tryAxis(box1, box2, n4, toCentre, 3, pen, best))   return false;
	if (!tryAxis(box1, box2, n5, toCentre, 4, pen, best))   return false;
	if (!tryAxis(box1, box2, n6, toCentre, 5, pen, best))   return false;

	unsigned bestSingleAxis = best;
	if (!tryAxis(box1, box2, n7, toCentre, 6, pen, best))   return false;
	if (!tryAxis(box1, box2, n8, toCentre, 7, pen, best))   return false;
	if (!tryAxis(box1, box2, n9, toCentre, 8, pen, best))   return false;
	if (!tryAxis(box1, box2, n10, toCentre, 9, pen, best))  return false;
	if (!tryAxis(box1, box2, n11, toCentre, 10, pen, best)) return false;
	if (!tryAxis(box1, box2, n12, toCentre, 11, pen, best)) return false;
	if (!tryAxis(box1, box2, n13, toCentre, 12, pen, best)) return false;
	if (!tryAxis(box1, box2, n14, toCentre, 13, pen, best)) return false;
	if (!tryAxis(box1, box2, n15, toCentre, 14, pen, best)) return false;

	if (best == 160.0f)
		return false;
	

	
	if (best < 3)
	{
		
		fillPointFaceBoxBox(box1, box2, toCentre, data, best, pen);
		
		return 1;
	}
	else if (best < 6)
	{
		// We've got a vertex of box one on a face of box two.
		// We use the same algorithm as above, but swap around
		// one and two (and therefore also the vector between their
		// centres).
		fillPointFaceBoxBox(box2, box1, toCentre * -1.0f, data, best - 3, pen);
		return 1;
	}
	else
	{
		// We've got an edge-edge contact. Find out which axes
		best -= 6;
		unsigned oneAxisIndex = best / 3;
		unsigned twoAxisIndex = best % 3;
	    glm::vec3 oneAxis = box1.getAxis(oneAxisIndex);
		glm::vec3 twoAxis = box2.getAxis(twoAxisIndex);
		glm::vec3 axis = glm::cross(oneAxis , twoAxis);
		axis = glm::normalize(axis);

		// The axis should point from box one to box two.
		if (glm::dot(axis , toCentre) > 0) axis = axis * -1.0f;

		// We have the axes, but not the edges: each axis has 4 edges parallel
		// to it, we need to find which of the 4 for each object. We do
		// that by finding the point in the centre of the edge. We know
		// its component in the direction of the box's collision axis is zero
		// (its a mid-point) and we determine which of the extremes in each
		// of the other axes is closest.
		glm::vec3 ptOnOneEdge = box1.halfSize();
		glm::vec3 ptOnTwoEdge = box2.halfSize();
		for (unsigned i = 0; i < 3; i++)
		{
			if (i == oneAxisIndex) ptOnOneEdge[i] = 0;
			else if (glm::dot(box1.getAxis(i) , axis) > 0) ptOnOneEdge[i] = -ptOnOneEdge[i];

			if (i == twoAxisIndex) ptOnTwoEdge[i] = 0;
			else if (glm::dot(box2.getAxis(i) , axis) < 0) ptOnTwoEdge[i] = -ptOnTwoEdge[i];
		}

		// Move them into world coordinates (they are already oriented
		// correctly, since they have been derived from the axes).
		ptOnOneEdge = glm::vec3(box1.body->getModelMatrix() * glm::vec4(ptOnOneEdge,1.0f));
		ptOnTwoEdge = glm::vec3(box2.body->getModelMatrix() * glm::vec4(ptOnTwoEdge, 1.0f));

		// So we have a point and a direction for the colliding edges.
		// We need to find out point of closest approach of the two
		// line-segments.
		glm::vec3 vertex = contactPoint( 
			ptOnOneEdge, oneAxis, box1.halfSize()[oneAxisIndex],
			ptOnTwoEdge, twoAxis, box2.halfSize()[twoAxisIndex],
			bestSingleAxis > 2
		);

		// We can fill the contact.
		Contact* contact = new Contact;

		contact->penetration = pen;
		contact->contactNormal = axis;
		contact->contactPoints.push_back(vertex);
		contact->setBodyData(box1.body, box2.body, 0.0f, 0.4f);
		return 1;
	}
	return 0;
}

void CollisionDetector::fillPointFaceBoxBox(const AABB& box1, const AABB& box2, const glm::vec3& toCentre, CollisionData* data, unsigned best, float pen)
{

	glm::vec3 normal = glm::vec3(0.0f);
	glm::vec3 kek;
	if (best == 0)
		normal = box1.getNormX();

	if (best == 1)
		normal = box1.getNormY();

	if (best == 2)
        normal = box1.getNormZ();


	if (glm::dot(normal, toCentre) > 0)
	{
		normal = normal * -1.0f;
	}

	
	glm::vec3 vertex = box2.halfSize();
	if (glm::dot(box2.getNormX() ,normal) < 0) vertex.x = -vertex.x;
	if (glm::dot(box2.getNormY(), normal) < 0) vertex.y = -vertex.y;
	if (glm::dot(box2.getNormZ(), normal) < 0) vertex.z = -vertex.z;

	
	Contact* contact = new Contact;
	contact->contactNormal = normal;
	contact->penetration = pen;
	contact->contactPoints.push_back(glm::vec3(box2.body->getModelMatrix() * glm::vec4(vertex, 1.0f)));
	contact->setBodyData(box1.body, box2.body, 0.0f, 0.4f);
	data->contactArray.push_back(std::move(contact));
}

glm::vec3 CollisionDetector::contactPoint(const glm::vec3& pOne, const glm::vec3& dOne, float oneSize, const glm::vec3& pTwo, const glm::vec3& dTwo, float twoSize, bool useOne)
{
	glm::vec3 toSt, cOne, cTwo;
	float dpStaOne, dpStaTwo, dpOneTwo, smOne, smTwo;
	float denom, mua, mub;

	smOne = glm::length(dOne);
	smTwo = glm::length(dTwo);
	dpOneTwo = glm::dot(dTwo , dOne);

	toSt = pOne - pTwo;
	dpStaOne = glm::dot(dOne , toSt);
	dpStaTwo = glm::dot(dTwo , toSt);

	denom = smOne * smTwo - dpOneTwo * dpOneTwo;


	if (fabs(denom) < 0.0001f) {
		return useOne ? pOne : pTwo;
	}

	mua = (dpOneTwo * dpStaTwo - smTwo * dpStaOne) / denom;
	mub = (smOne * dpStaTwo - dpOneTwo * dpStaOne) / denom;

	
	if (mua > oneSize ||
		mua < -oneSize ||
		mub > twoSize ||
		mub < -twoSize)
	{
		return useOne ? pOne : pTwo;
	}
	else
	{
		cOne = pOne + dOne * mua;
		cTwo = pTwo + dTwo * mub;

		return cOne * 0.5f + cTwo * 0.5f;
	}
}


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

	float len1 = (resultMax1 - resultMin1);
	float len2 = (resultMax2 - resultMin2);
	float min = fminf(resultMin1, resultMin2);
	float max = fmaxf(resultMax1, resultMax2);
	float length = (max - min);

	if (isFlip != false) {
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

	
	normals.push_back(glm::cross(normals[0], normals[3]));
	normals.push_back(glm::cross(normals[0], normals[4]));
	normals.push_back(glm::cross(normals[0], normals[5]));

	normals.push_back(glm::cross(normals[1], normals[3]));
	normals.push_back(glm::cross(normals[1], normals[4]));	
	normals.push_back(glm::cross(normals[1], normals[5]));


	normals.push_back(glm::cross(normals[2], normals[3]));
	normals.push_back(glm::cross(normals[2], normals[4]));
	normals.push_back(glm::cross(normals[2], normals[5]));




	bool isflip;
	unsigned best = 0;
	float penetration = 100000.0f;
	glm::vec3 hitNormal;
	for (int i = 0; i < normals.size(); ++i)
	{
		if (normals[i].x < 0.00001f) normals[i].x = 0.0f;
		if (normals[i].y < 0.00001f) normals[i].y = 0.0f;
		if (normals[i].z < 0.00001f) normals[i].z = 0.0f;
		if (glm::length(normals[i]) < 0.001f) {
			continue;
		}
		if (!tryAxis(box1, box2, normals[i], toCenter, i, penetration, best))  return false;

		//float depth = penetrationOnAxis(box1, box2, normals[i], &isflip);
		//if (depth <= 0.0f)
		//	return false;
		//else if (depth < penetration) {
		//	if (isflip) {
		//		normals[i] = normals[i] * -1.0f;
		//	}
		//	penetration = depth;
		//	hitNormal = normals[i];
		//	best = i;
		//}
		
	}
	hitNormal = normals[best];
	if (hitNormal == glm::vec3(0.0f))
		return false;
	
	std::cout << best << std::endl;
	glm::vec3 axis = glm::normalize(hitNormal);

	Contact* contact = new Contact;
	contact->contactNormal = axis;
	contact->penetration = penetration;

	std::vector<glm::vec3> maxVertexes1;
	float max1 = glm::length(box1.halfSize() * axis);

	glm::vec3 maxPoint;
	for (int i = 0; i < 8; ++i)
	{
		float temp = glm::dot(axis, VerBox1[i] - box1.getPosition());
		if (temp >= max1)
		{
			max1 = temp;
			maxPoint = VerBox1[i];
		}
	}
	for (int i = 0; i < 8; ++i)
	{
		float temp = glm::dot(axis, VerBox1[i] - box1.getPosition());
		if (fabs(temp - max1) <= 0.1f || temp == max1)
			maxVertexes1.push_back(VerBox1[i]);
	}

	std::vector<glm::vec3> maxVertexes2;

	float max2 = glm::length(box2.halfSize() * -axis);
	for (int i = 0; i < 8; ++i)
	{
 		float temp = glm::dot(-axis, VerBox2[i] - box2.getPosition());
		if (temp >= max2)
			max2 = temp;
	}
	for (int i = 0; i < 8; ++i)
	{
		float temp = glm::dot(-axis, VerBox2[i] - box2.getPosition());
		if (fabs(temp - max2) <= 0.1f || temp == max2)
			maxVertexes2.push_back(VerBox2[i]);
	}

   	glm::vec3 mostPerpendicular;
	
	float xProjection = fabs(glm::dot(axis , glm::vec3(1.0f, 0.0f, 0.0f))); 
	float yProjection = fabs(glm::dot(axis , glm::vec3(0.0f, 1.0f, 0.0f))); 
	float zProjection = fabs(glm::dot(axis , glm::vec3(0.0f, 0.0f, 1.0f))); 
	if ((xProjection > yProjection) && (xProjection > zProjection)) mostPerpendicular = glm::vec3(1.0f, 0.0f, 0.0f);
	else
		if ((yProjection > xProjection) && (yProjection > zProjection)) mostPerpendicular = glm::vec3(0.0f, 1.0f, 0.0f);
		else
			mostPerpendicular = glm::vec3(0.0f, 0.0f, 1.0f);

	glm::vec3 clipAxisX = glm::normalize(glm::cross(axis, mostPerpendicular));
	if (axis == glm::vec3(glm::vec3(1.0f, 0.0f, 0.0f)))
		clipAxisX = glm::vec3(0.0f, 1.0f, 0.0f);
	if (axis == glm::vec3(glm::vec3(0.0f, 1.0f, 0.0f)))
		clipAxisX = glm::vec3(1.0f, 0.0f, 0.0f);
	if (axis == glm::vec3(glm::vec3(0.0f, 0.0f, 1.0f)))
		clipAxisX = glm::vec3(0.0f, 1.0f, 0.0f);

	glm::vec3 clipAxisY = glm::normalize(glm::cross(clipAxisX, axis));

	std::vector<glm::vec2> clippedPoints1;
	std::vector<glm::vec2> clippedPoints2;
	for (int i = 0; i < maxVertexes1.size(); ++i)
	{
		float  point1 = glm::dot(maxVertexes1[i], clipAxisX);
		float  point2 = glm::dot(maxVertexes1[i], clipAxisY);
		clippedPoints1.push_back(glm::vec2(point1, point2));
	}
	
 	for (int i = 0; i < maxVertexes2.size(); ++i)
	{
		float  point1 = glm::dot(maxVertexes2[i], clipAxisX);
		float  point2 = glm::dot(maxVertexes2[i], clipAxisY);
		clippedPoints2.push_back(glm::vec2(point1, point2));
	}

   	std::vector<glm::vec2> contacts = Geometry3D::intersectionOfPolyhedrons(clippedPoints1,clippedPoints2);

	std::vector<glm::vec3> resultContact;
	glm::mat4x4 toVec3Mat = glm::mat4x4(1.0f);
	toVec3Mat[0] =  glm::vec4(clipAxisX, 1.0f);
	toVec3Mat[1] =  glm::vec4(clipAxisY, 1.0f);
	toVec3Mat[2] =  glm::vec4(axis, 1.0f);


	for (int i = 0; i < contacts.size(); ++i)
	{
  		glm::vec3 current = glm::vec3(contacts[i] , glm::dot(maxPoint, axis));
		current = glm::vec3(toVec3Mat * glm::vec4(current, 1.0f));
  		resultContact.push_back(current);
	}
	
	for (int i = 0; i < resultContact.size(); ++i)
	{
		contact->contactPoints.push_back(resultContact[i]);
	}
	for (int i = contact->contactPoints.size() - 1; i >= 0; --i)
	{
		for (int j = contact->contactPoints.size() - 1; j > i; --j)
		{
			if (glm::length(contact->contactPoints[i] - contact->contactPoints[j]) < 0.001f)
			{
				contact->contactPoints.erase(contact->contactPoints.begin() + j);
				break;
			}
		}
	}
   	contact->setBodyData(box2.body, box1.body, 0.0f,0.5f);
	data->contactArray.push_back(std::move(contact));
	return true;
}

std::vector<glm::vec3> CollisionDetector::clipEdges(const std::vector<Line>& edges, const std::vector<Plane> & planes, const AABB& box)
{
	std::vector<glm::vec3> result;

	glm::vec3 intersection;
	for (int i = 0; i < planes.size(); ++i) {
		for (int j = 0; j < edges.size(); ++j) {
			if (clipPlane(edges[j], planes[i], &intersection,box)) 
			{
				if(PointInOBB(intersection, box)) 
				{
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
	for (int i = 0; i < 3; ++i)
	{
		float dist = glm::dot(dir, normals[i]);
		if (dist > half[i])
			return false;
		if (dist < -half[i])
			return false;
	}
	return true;
}


bool CollisionDetector::clipPlane(const Line &edge, const Plane &plane, glm::vec3* intersectionPoint, const AABB& box)
{
	glm::vec3 ab =  edge.end - edge.start;

	float nA = glm::dot(plane.normal, edge.start);
	float nAB = glm::dot(plane.normal, ab);

	if (nAB == 0.0f) {
		return false;
	}

	float t = (plane.offset - nA) / nAB;
	if (t >= 0.0f && t <= 1.0f) 
	{
		if (intersectionPoint != nullptr) {
			*intersectionPoint = edge.start + t * ab;
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
	
		contact->setBodyData(aabb.body, plane.body, 0.0f, 0.5f);
		data->contactArray.push_back(std::move(contact));
	
	}

	return collide;


}

