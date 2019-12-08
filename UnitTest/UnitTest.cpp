#include "pch.h"
#include "CppUnitTest.h"
#include "../3DphysEngine/include/RigidBody.h"
#include "../3DphysEngine/src/RigidBody.cpp"
#include "../glm/glm/glm.hpp"
#include <cmath>


using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace UnitTest
{
	TEST_CLASS(RigidBody_TEST)
	{
	public:
		TEST_METHOD(BoxTensorInertiaTest)
		{
			RigidBody test(RIGIDBODY_TYPE_BOX, 1.0f);
			glm::mat3x3 tensor = test.getInvInersiaTensor();
			Assert::IsTrue(std::fabs(tensor[0][0] - 1.2f) < 0.0001f);
			Assert::IsTrue(std::fabs(tensor[1][1] - 1.2f) < 0.0001f);
			Assert::IsTrue(std::fabs(tensor[2][2] - 1.2f) < 0.0001f);
		}
		TEST_METHOD(SphereTensorInertiaTest)
		{
			RigidBody test(RIGIDBODY_TYPE_SPHERE, 1.0f);
			glm::mat3x3 tensor = test.getInvInersiaTensor();
			Assert::IsTrue(std::fabs(tensor[0][0] - 2.0f) < 0.0001f);
			Assert::IsTrue(std::fabs(tensor[1][1] - 2.0f) < 0.0001f);
			Assert::IsTrue(std::fabs(tensor[2][2] - 2.0f) < 0.0001f);
		}
		TEST_METHOD(PlaneTensorInertiaTest)
		{
			RigidBody test(RIGIDBODY_TYPE_PLANE, 0.0f);
			glm::mat3x3 tensor = test.getInvInersiaTensor();
			Assert::IsTrue(tensor[0][0] == 0.0f);
			Assert::IsTrue(tensor[1][1] == 0.0f);
			Assert::IsTrue(tensor[2][2] == 0.0f);
		}
		TEST_METHOD(NaNPlaneTensorInertiaTest)
		{
			RigidBody test(RIGIDBODY_TYPE_PLANE, 1.0f);
			glm::mat3x3 tensor = test.getInvInersiaTensor();
			Assert::IsTrue(std::isnan(tensor[0][0]));
			Assert::IsTrue(std::isnan(tensor[1][1]));
			Assert::IsTrue(std::isnan(tensor[2][2]));
		}
		TEST_METHOD(AddImpuleTest)
		{
			RigidBody test(RIGIDBODY_TYPE_SPHERE, 1.0f);
			test.AddLinearImpulse(glm::vec3(0.0f, 0.0f, 4.0f));
			glm::vec3 v = test.getVelocity();
			Assert::IsTrue(std::fabs(v.z - 4.0f) < 0.0001f);
			Assert::IsTrue(std::fabs(v.y - 0.0f) < 0.0001f);
			Assert::IsTrue(std::fabs(v.x - 0.0f) < 0.0001f);
		}
		TEST_METHOD(AddTorqueTest)
		{
			RigidBody test(RIGIDBODY_TYPE_SPHERE, 1.0f);
			test.addTorque(glm::vec3(0.0f, 0.0f, 4.0f));
			test.update(1 / 60.0f);
			glm::vec3 r = test.getRotation();
			Assert::IsTrue(std::fabs(r.z - 0.1293f) < 0.0001f);
			Assert::IsTrue(std::fabs(r.y - 0.0f) < 0.0001f);
			Assert::IsTrue(std::fabs(r.x - 0.0f) < 0.0001f);
		}
		TEST_METHOD(AddImpuleInPointTest)
		{
			RigidBody test(RIGIDBODY_TYPE_SPHERE, 1.0f);
			test.AddRotationalImpulse(glm::vec3(1.0f, 0.0f, 0.0f),glm::vec3(0.0f, 0.0f, 4.0f));
			glm::vec3 v = test.getVelocity();
			glm::vec3 r = test.getRotation();
			Assert::IsTrue(std::fabs(v.z - 4.0f) < 0.0001f);
			Assert::IsTrue(std::fabs(v.y - 0.0f) < 0.0001f);
			Assert::IsTrue(std::fabs(v.x - 0.0f) < 0.0001f);

			Assert::IsTrue(std::fabs(r.z - 4.0f) < 0.0001f);
			Assert::IsTrue(std::fabs(r.y - 0.0f) < 0.0001f);
			Assert::IsTrue(std::fabs(r.x - 0.0f) < 0.0001f);
		}
	};
	TEST_CLASS(CollisionDetector_TEST)
	{
	public:
		TEST_METHOD(CubeVsCubeTest)
		{

		}
		TEST_METHOD(SphereVsSphereTest)
		{

		}
	};
}
