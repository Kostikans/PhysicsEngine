#version 330 core
out vec4 FragColor;

uniform vec3 AxisColor;


void main()
{
	FragColor = glm::vec4(AxisColor,1.0f);
}