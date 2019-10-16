#pragma once
#include "CollisionData.h"
#include "Geometry3D.h"
#include "Sphere.h"
#include "AABB.h"
#include "Plain.h"
struct Line {
public:
	glm::vec3 start;
	glm::vec3 end;
	Line(glm::vec3 m_start, glm::vec3 m_end)
	{
		start = m_start;
		end = m_end;
	}
};
struct Plane
{
public:
	float offset;
	glm::vec3 normal;
	Plane(glm::vec3 m_normal,float m_offset)
	{
		offset = m_offset;
		normal = m_normal;
	}
};
class CollisionDetector
{
public:
	static bool boxAndSphere(const AABB& aabb, const Sphere& sphere, CollisionData* data);
	static bool boxAndPlain(const AABB& aabb, const Plain& plane, CollisionData* data);
	static bool sphereAndTruePlane(const Sphere& sphere, const Plain& plane, CollisionData* data);
	static bool sphereAndSphere(const Sphere& one, const Sphere& two, CollisionData* data);


	static float penetrationOnAxis(const AABB& box1, const AABB& box2, const glm::vec3& axis, bool *isFlip);
	static bool boxVsBox(const AABB& box1, const AABB& box2, CollisionData* data);
	static std::vector<glm::vec3> clipEdges(const std::vector<Line>& edges, const std::vector<Plane> &planes, const AABB& box);
	static bool clipPlane(const Line &edge, const Plane &plane, glm::vec3 *intersectionPoint);
	static bool PointInOBB(const glm::vec3& point, const AABB& obb);
};