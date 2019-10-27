#pragma once
#include "CollisionData.h"
#include "Geometry3D.h"
#include "Sphere.h"
#include "AABB.h"
#include "Plain.h"
#include "Contact.h"


struct Plane
{
public:
	float offset;
	glm::vec3 normal;
	Plane(const glm::vec3 &m_normal,float m_offset)
		:offset(m_offset),normal(m_normal)
	{
	}
};
class CollisionDetector
{
public:
	static std::vector<glm::vec3> separationAxes;
	static  glm::mat4x4 model;
	static bool boxAndSphere(const AABB& aabb, const Sphere& sphere, CollisionData* data);
	static bool boxAndPlain(const AABB& aabb, const Plain& Plain, CollisionData* data);
	static bool sphereAndTruePlain(const Sphere& sphere, const Plain& Plain, CollisionData* data);
	static bool sphereAndSphere(const Sphere& one, const Sphere& two, CollisionData* data);

	static float penetrationOnAxis(glm::vec3* VerBox1, glm::vec3* VerBox2, const AABB& box1, const AABB& box2, glm::vec3& axis, bool *isFlip);
	static bool boxVsBox(const AABB& box1, const AABB& box2, CollisionData* data);

	static float penetrationOnAxis(const AABB& box1, const AABB& box2, const glm::vec3& axis, const glm::vec3& toCentre);
	static float transformToAxis(const AABB& box, const glm::vec3& axis);
	static bool tryAxis(const AABB& box1, const AABB& box2,  glm::vec3& axis, const glm::vec3& toCentre, unsigned index, float& smallestPenetration, unsigned& smallestCase);
	
	static void  getSepAxis( std::vector<glm::vec3> sep, const glm::vec3 &pos);
	static void drawSepAxis(Shader& shader);

};