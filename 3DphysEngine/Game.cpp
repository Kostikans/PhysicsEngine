#include <iostream>

#define GLEW_STATIC
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <chrono>
#include <thread>

#include "include/Transformation.h"
#include "include/Camera.h"
#include "include/CollisionDetector.h"
#include "include/Gravity.h"
#include "include/CollisionData.h"
#include "include/EngineRoutine.h"
#include "gtc/constants.hpp"


void mouse_callback(GLFWwindow* window, double xpos, double ypos);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void processInput(GLFWwindow* window);

const GLint WIDTH = 1600, HEIGHT = 960;

Camera camera(glm::vec3(glm::vec3(0.0f, 7.0f, 3.0f)));
bool firstMouse = true;

float lastX = WIDTH / 2.0f;
float lastY = HEIGHT / 2.0f;

float deltaTime = 0.0f;
float lastFrame = 0.0f;

bool debug = false;
bool rot = false;

bool up = false;
bool down = false;
bool left = false;
bool right = false;
bool jump = false;
int main()
{
	
	GLFWwindow* window;
	if (!glfwInit())
		return -1;
	
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	glfwWindowHint(GLFW_SAMPLES, 16);

	window = glfwCreateWindow(WIDTH, HEIGHT, "Engine", glfwGetPrimaryMonitor(), NULL);

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

	Shader debugShader("Shaders//Debug.vs", "Shaders//Debug.fs");
	Shader shader("Shaders//Vertex.vs", "Shaders//Fragment.fs");
	Shader ContactPoint("Shaders//ContactPoint.vs", "Shaders//ContactPoint.fs");

	

	glm::mat4x4 projectionMatrix = glm::mat4x4(1.0f);
	projectionMatrix = glm::perspective(glm::radians(45.0f), (float)WIDTH / (float)HEIGHT, 0.1f, 100.0f);
	shader.setMat4("projectionMatrix", projectionMatrix);


	Gravity gravity;

	std::vector<Transformation*> obj;
	Transformation *plane = new Plain(0.0f);
	plane->init(40.0f, 40.0f,0.0f);
	//plane->rotate(glm::quat(glm::angleAxis(glm::radians(15.0f), glm::vec3(1.0, 0.0, 0.0))));
	plane->translate(glm::vec3(0.0f, -7.0f, -3.0f));
	obj.push_back(plane);

	Transformation* sph = new Sphere(7.0f);

	sph->init(1.0f, 0.0f, 0.0f);
	sph->translate(glm::vec3(12.0f, -3.0f, -6.0f));
	//sph->addImpulse(glm::vec3(2.0f, 3.0f, 35.f));
	obj.push_back(sph);

	/*for (int i = 0; i < 10; ++i)
	{	
			Transformation* aabb = new AABB(5.0f);
			aabb->init(1.0f, 1.0f, 1.0f);
			aabb->translate(glm::vec3((4.0f) * 1.1f, (0.0f + i * 3.0f),  1.1f));
			obj.push_back(aabb);
	}*/
	for (int i = 0; i < 2; ++i)
	{
		Transformation* aabb = new AABB(4.0f);
		aabb->init(1.0f, 1.0f, 1.0f);
		aabb->translate(glm::vec3(1.7f + i * 1.6f, -4.0f + 2.5 * i, 0.0f));
		obj.push_back(aabb);
	}
	obj[3]->addTorque(glm::vec3(1000.0f, 0.0f, 0.0f));
	/*for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			Transformation* aabb = new AABB(5.0f);
			aabb->init(1.0f, 1.0f, 1.0f);
			aabb->translate(glm::vec3(i * 2.0f, (-6.0f + j * 4.0f), 0.0f));
			obj.push_back(aabb);
		}
	}*/
	/*int pyro = 3;
	float offset = 0.0f;
	for (int i = 0; i < 3; ++i)
	{
		for (int j = pyro; j > 0; --j)
		{
			Transformation* aabb = new AABB(1.0f);
			aabb->init(1.0f, 1.0f, 1.0f);
			aabb->translate(glm::vec3((j + 4.0f + offset) * 2.0f, (-6.0f + i * 2.0f), 5.0f));
			obj.push_back(aabb);
			
		}
		offset += 0.5f;
		--pyro;
	}*/
	 //kek->body->addTorque(glm::vec3(.0f, 0.0f,.0f));
     //kek.body->AddLinearImpulse(glm::vec3(0.0f, 0.f, -6.0f));
     //obj[2]->addTorque(glm::vec3(000.f, 500.0f, 000.0f));
	 float deltaPhys = 1.0f / 60.0f;

	 EngineRoutine physics;
	 
	 for (int i = 0; i < obj.size(); ++i)
	 {
		 obj[i]->move(deltaPhys);
		 physics.addEntity(obj[i]);
	 }
	 int width, height;
	 glfwGetWindowSize(window, &width, &height);
	 glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

	 glfwSwapInterval(1);

	 while (!glfwWindowShouldClose(window))
	 {
		 processInput(window);


		 projectionMatrix = glm::perspective(glm::radians(45.0f), (float)width / (float)height, 0.1f, 100.0f);

		 float currentFrame = glfwGetTime();
		 deltaTime = currentFrame - lastFrame;
		 lastFrame = currentFrame;

		 glClearColor(0.05f, 0.05f, 0.05f, 1.0f);
		 glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		 ContactPoint.use();
		 ContactPoint.setMat4("projectionMatrix", projectionMatrix);
		 camera.setViewMatrix(ContactPoint);
		 if (rot == true)
		 {
			 obj[0]->rotate(glm::quat(glm::angleAxis(glm::radians(deltaTime), glm::vec3(1.0, 0.0, 0.0))));
		 }
		 if (debug == true)
         {
		    physics.run(deltaPhys,debugShader,ContactPoint);
		 }
		 if (up == true)
		 {
			obj[1]->addImpulse(glm::vec3(0.0f, 0.0f, -0.5f));
		 }
		 if (left == true)
		 {
			obj[1]->addImpulse(glm::vec3(-0.5f, 0.0f, 0.0));
		 }
		 if (right == true)
		 {
			obj[1]->addImpulse(glm::vec3(0.5f, 0.0f, 0.00f));
		 }
		 if (down == true)
		 {
			obj[1]->addImpulse(glm::vec3(0.0f, 0.0f, 0.5f));
		 }
		 debugShader.use();
		 debugShader.setMat4("projectionMatrix", projectionMatrix);
		 camera.setViewMatrix(debugShader);
		 for (int i = 0; i < obj.size(); ++i)
		 {
			 obj[i]->drawNormal(debugShader);
		 }

		 shader.use();
		 shader.setMat4("projectionMatrix", projectionMatrix);
		 camera.setViewMatrix(shader);

		 for (int i = 0; i < obj.size(); ++i)
		 {
			 obj[i]->draw(shader,deltaTime);
		 }
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

	if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS)
		up = true;
	else
		up = false;
	if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS)
		left = true;
	else
		left = false;
	if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS)
		right = true;
	else
		right = false;
	if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS)
		down = true;
	else
		down = false;

	if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS)
	{
		debug = true;
		jump = true;
	}
	else
	{
		debug = false;
		jump = false;
	}

	if (glfwGetKey(window, GLFW_KEY_ENTER) == GLFW_PRESS)
		rot = true;
	else
		rot = false;
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

void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
	glViewport(0, 0, width, height);
}

