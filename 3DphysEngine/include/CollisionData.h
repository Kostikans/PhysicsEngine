#pragma once

#include "Contact.h"
#include "Shader.h"
#include "Transformation.h"

class CollisionData
{
public:
	std::vector<Contact*> contactArray;

	int contactsLeft;
	unsigned contactCount;
	float friction;
	float restitution;
	float tolerance;

	CollisionData() {}

	void contactPointView(Shader& shader)
	{
		for (int i = 0; i < contactArray.size(); ++i)
		{
			shader.use();
			VertexBuffer vbo;
			vbo.allocate(contactArray[i]->contactPoints.data(), contactArray[i]->contactPoints.size() * sizeof(glm::vec3));

			VertexAttribBuffer vao;
			AttribLayout layout1;
			layout1.push<float>(3);
			vao.pushLayout(layout1, vbo);
			shader.setMat4("modelMatrix", contactArray[i]->body[1]->getModelMatrix());
			glm::vec3 color = glm::vec3(1.0f, 0.0f, 0.0);
			glPointSize((GLfloat)10.0f);
			shader.setVec3("AxisColor", color);

			vbo.bind();
			glDrawArrays(GL_POINTS, 0, contactArray[i]->contactPoints.size());
			vbo.unbind();
		}
	}
};