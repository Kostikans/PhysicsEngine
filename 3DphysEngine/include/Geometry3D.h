#pragma once

#include "Transformation.h"
#include "AABB.h"
#include "Sphere.h"
#include "Plain.h"

class Geometry3D
{
public:
	static glm::vec3 ClosestPoint(const AABB& aabb, const glm::vec3& point)
	{
		glm::vec3 result = point;
		glm::vec3 min = aabb.getMin();
		glm::vec3 max = aabb.getMax();

		result.x = (result.x < min.x) ? min.x : result.x;
		result.y = (result.y < min.y) ? min.y : result.y;
		result.z = (result.z > min.z) ? min.z : result.z;

		result.x = (result.x > max.x) ? max.x : result.x;
		result.y = (result.y > max.y) ? max.y : result.y;
		result.z = (result.z < max.z) ? max.z : result.z;

		return result;
	}
	static glm::vec3 ClosestPoint(const Plain& plain, const glm::vec3& point)
	{
		glm::vec3 result = point;
		glm::vec3 min = plain.getMin();
		glm::vec3 max = plain.getMax();

		result.x = (result.x < min.x) ? min.x : result.x;
		result.y = (result.y < min.y) ? min.y : result.y;
		result.z = (result.z < min.z) ? min.z : result.z;

		result.x = (result.x > max.x) ? max.x : result.x;
		result.y = (result.y > max.y) ? max.y : result.y;
		result.z = (result.z > max.z) ? max.z : result.z;

		return result;
	}
	static bool pointInAABB(const AABB& aabb, const glm::vec3& point)
	{
		glm::vec3 min = aabb.getMin();
		glm::vec3 max = aabb.getMax();
		if (point.x < min.x || point.y < min.y || point.z < min.z)
			return false;
		if (point.x > max.x || point.y > max.y || point.z > max.z)
			return false;
		return true;
	}
};