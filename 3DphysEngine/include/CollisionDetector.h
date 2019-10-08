#pragma once
#include "CollisionData.h"
#include "Geometry3D.h"
#include "Sphere.h"
#include "AABB.h"
#include "Plain.h"

class CollisionDetector
{
public:
	static bool boxAndSphere(const AABB& aabb, const Sphere& sphere, CollisionData* data);
	static bool boxAndPlain(const AABB& aabb, const Plain& plane, CollisionData* data);
	static bool sphereAndTruePlane(const Sphere& sphere, const Plain& plane, CollisionData* data);
	static bool sphereAndSphere(const Sphere& one, const Sphere& two, CollisionData* data);
	static bool boxAndBox(const AABB& box1, const AABB& box2, CollisionData* data);
	static bool overLapOnAxis(const AABB& box1, const AABB& box2, const glm::vec3 axis);
	static float transformToAxis(const AABB& box, const glm::vec3 axis);
	static float sgn(float& x);
};