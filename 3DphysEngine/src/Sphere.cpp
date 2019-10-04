#define _USE_MATH_DEFINES
#include <math.h>
#include "..\include\Sphere.h"


Sphere::Sphere()
	: m_scale(1.0f)
{
	texture = new Texture("E:\\download\\45614125-bcb2ce00-ba68-11e8-903b-f22addb83ca5.jpg");
	body = new RigidBody(RIGIDBODY_TYPE_SPHERE,1.0f);
}

void Sphere::draw(Shader& shader, float deltaTime)
{
	shader.use();
	vao.bind();
	texture->bind();
	shader.setInt("texture1", 0);
	AttribLayout layout;
	layout.push<float>(3);
	layout.push<float>(2);
	layout.push<float>(3);
	vao.pushLayout(layout, vbo);

	shader.setMat4("modelMatrix", body->getModelMatrix());
	
	ibo.bind();
	glDrawElements(GL_TRIANGLES, ibo.size(), GL_UNSIGNED_INT, 0);
	ibo.unbind();


}

void Sphere::translate(const glm::vec3& translate)
{
	m_translate += translate;
	body->setPosition(m_translate);
}

void Sphere::scale(const float& scale)
{
	m_scale *= scale;
}

void Sphere::rotate(const glm::quat& rotate)
{
	m_rotate *= rotate;
}

void Sphere::init(float radius)
{
	m_radius = radius;
	std::vector<VertexData> vertexes;
	float sectorCount = 72;
	float stackCount = 72;
	float x, y, z, xy;
	float nx, ny, nz, lengthInv = 1.0f / radius;
	float s, t;

	float sectorStep = 2 * M_PI / sectorCount;
	float stackStep = M_PI / stackCount;
	float sectorAngle, stackAngle;

	for (int i = 0; i <= stackCount; ++i)
	{
		stackAngle = M_PI / 2 - i * stackStep;
		xy = radius * cosf(stackAngle);
		z = radius * sinf(stackAngle);

		for (int j = 0; j <= sectorCount; ++j)
		{
			sectorAngle = j * sectorStep;

			x = xy * cosf(sectorAngle);
			y = xy * sinf(sectorAngle);

			nx = x * lengthInv;
			ny = y * lengthInv;
			nz = z * lengthInv;

			s = (float)j / sectorCount;
			t = (float)i / stackCount;

			vertexes.push_back(VertexData(glm::vec3(x, y, z),glm::vec2(s,t),glm::vec3(nx,ny,nz)));
		}
	}
	std::vector<GLuint> indices;
	int k1, k2;
	for (int i = 0; i < stackCount; ++i)
	{
		k1 = i * (sectorCount + 1);
		k2 = k1 + sectorCount + 1;

		for (int j = 0; j < sectorCount; ++j, ++k1, ++k2)
		{
			if (i != 0)
			{
				indices.push_back(k1);
				indices.push_back(k2);
				indices.push_back(k1 + 1);
			}
			if (i != (stackCount - 1))
			{
				indices.push_back(k1 + 1);
				indices.push_back(k2);
				indices.push_back(k2 + 1);
			}
		}
	}
	
	vbo.allocate(vertexes.data(), vertexes.size() * sizeof(VertexData));
	ibo.allocate(indices.data(), indices.size() * sizeof(GLuint));
	
}

void Sphere::move(float deltaTime)
{
	body->update(deltaTime);
}

float Sphere::getRadius() const
{
	return m_radius;
}

glm::vec3 Sphere::getPosition() const
{
	return body->getPosition();
}
