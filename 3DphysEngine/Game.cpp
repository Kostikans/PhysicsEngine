#include <iostream>

#define GLEW_STATIC
#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include "include/Sphere.h"
#include "include/Camera.h"
#include "include/Plain.h"
#include "include/AABB.h"
#include "include/CollisionDetector.h"
#include "include/Gravity.h"
#include "include/CollisionData.h"
#include "include/EngineRoutine.h"

void mouse_callback(GLFWwindow* window, double xpos, double ypos);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
void processInput(GLFWwindow* window);

const GLint WIDTH = 800, HEIGHT = 600;

Camera camera(glm::vec3(glm::vec3(0.0f, 7.0f, 3.0f)));
bool firstMouse = true;

float lastX = WIDTH / 2.0f;
float lastY = HEIGHT / 2.0f;

float deltaTime = 0.0f;
float lastFrame = 0.0f;
int main()
{
	
	GLFWwindow* window;
	if (!glfwInit())
		return -1;
	
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	glfwWindowHint(GLFW_SAMPLES, 16);

	window = glfwCreateWindow(WIDTH, HEIGHT, "Engine", NULL, NULL);

	if (!window)
	{
		glfwTerminate();
		return -1;
	}
	glfwMakeContextCurrent(window);
	glViewport(0, 0, WIDTH, HEIGHT);
	glewInit();

	glEnable(GL_DEPTH_TEST);
	glEnable(GL_MULTISAMPLE);

	glfwSetCursorPosCallback(window, mouse_callback);
	glfwSetScrollCallback(window, scroll_callback);
	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

	Shader shader("Shaders//Vertex.vs", "Shaders//Fragment.fs");
	

	glm::mat4x4 projectionMatrix = glm::mat4x4(1.0f);
	projectionMatrix = glm::perspective(glm::radians(45.0f), (float)WIDTH / (float)HEIGHT, 0.1f, 100.0f);
	shader.setMat4("projectionMatrix", projectionMatrix);



	Sphere sphere1;
	sphere1.init(1.0f);
	sphere1.translate(glm::vec3(-3.0f, 7.0f, -3.0f));
	sphere1.body->setMass(1.0f);


	Sphere sphere2;
	sphere2.init(1.0f);
	sphere2.translate(glm::vec3(3.0f, 7.0f, -3.0f));
	sphere2.body->setMass(1.0f);

	Gravity gravity;

	Plain plane;
	plane.init(40.0f, 40.0f);
	plane.translate(glm::vec3(0.0f, -7.0f, -3.0f));
    plane.body->setStat(false);

	
	 AABB aabb;
	 aabb.init(1.0f, 1.0f, 1.0f);
	 aabb.translate(glm::vec3(11.0f, 7.0f, -3.0f));
	 aabb.body->setMass(5.0f);


	//AABB kek;
   // kek.init(40.0f, 1.0f, 40.0f);
	//kek.translate(glm::vec3(0.0f, 0.0f, -3.0f));
	// kek.setMass(1.0f);
	//kek.body->setStat(true);


	 sphere1.body->AddLinearImpulse(glm::vec3(18.0f, 10.f, 0.0f));
	 sphere2.body->AddLinearImpulse(glm::vec3(-8.0f, 10.f, 0.0f));

	 

     //aabb.body->AddLinearImpulse(glm::vec3(-10.0f, 9.f, 0.0f));
	  aabb.body->AddRotationalImpulse(glm::vec3(0.5f, 7.f, 0.0f), glm::vec3(-3.0f, 100.f, 0.0f));
	
	 float deltaPhys = 1.0f / 40.0f;

	 CollisionData* data = new CollisionData;
	 EngineRoutine physics;

	 sphere1.body->update(deltaPhys);
	 sphere2.body->update(deltaPhys);
	 aabb.body->update(deltaPhys);


	while (!glfwWindowShouldClose(window))
	{
		float currentFrame = glfwGetTime();
		deltaTime = currentFrame - lastFrame;
		lastFrame = currentFrame;
		

		processInput(window);

		glClearColor(0.05f, 0.05f, 0.05f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		
		CollisionDetector::sphereAndSphere(sphere1, sphere2, data);

		CollisionDetector::sphereAndTruePlane(sphere1, plane, data);
		CollisionDetector::sphereAndTruePlane(sphere2, plane, data);

		CollisionDetector::boxAndPlain(aabb, plane, data);
		
		
		
		
	  
		CollisionDetector::boxAndSphere(aabb, sphere2, data);
	
		
	   CollisionDetector::boxAndSphere(aabb, sphere2, data);
	


	    gravity.updateGravity(sphere1.body);
	    gravity.updateGravity(sphere2.body);
	    gravity.updateGravity(aabb.body);
		
			if (data->contactArray.empty() == 0)
			{
				for (int i = 0; i < data->contactArray.size(); ++i)
				{
					physics.resolveContacts(data->contactArray[i], 1, deltaPhys);
				}

				data->contactArray.clear();
			}
		
		
		sphere2.move(deltaPhys);
		aabb.move(deltaPhys);
		sphere1.move(deltaPhys);
		//sphere2.move(deltaPhys);
		//aabb.move(deltaPhys);
		//plane.move(deltaPhys);

		shader.setMat4("projectionMatrix", projectionMatrix);
		camera.setViewMatrix(shader);
		
		sphere1.draw(shader, deltaPhys);
		sphere2.draw(shader, deltaPhys);
		
		aabb.draw(shader, deltaPhys);
		plane.draw(shader, deltaPhys);
	
	
		glfwSwapBuffers(window);
		glfwPollEvents();
	}

	glfwTerminate();
	return 0;

}

void processInput(GLFWwindow* window)
{
	if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
		glfwSetWindowShouldClose(window, true);

	if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
		camera.ProcessKeyboard(FORWARD, deltaTime);
	if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
		camera.ProcessKeyboard(BACKWARD, deltaTime);
	if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
		camera.ProcessKeyboard(LEFT, deltaTime);
	if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
		camera.ProcessKeyboard(RIGHT, deltaTime);
}

void mouse_callback(GLFWwindow* window, double xpos, double ypos)
{
	if (firstMouse)
	{
		lastX = xpos;
		lastY = ypos;
		firstMouse = false;
	}

	float xoffset = xpos - lastX;
	float yoffset = lastY - ypos;

	lastX = xpos;
	lastY = ypos;

	camera.ProcessMouseMovement(xoffset, yoffset);
}

void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
	camera.ProcessMouseScroll(yoffset);
}