#pragma once

#define GLEW_STATIC
#include <GL/glew.h>
#include <string>
#include <glm.hpp>

class Shader
{
private:
	unsigned int ID;
	void checkCompileErrors(GLuint shader, std::string type);

public:
	Shader(const char* vertexPath, const char* fragmentPath, const char *geometryPath = nullptr);
	void use();
	void setBool(const std::string& name, bool value) const;
	void setInt(const std::string& name, int value) const;
	void setFloat(const std::string& name, float value) const;
	void setVec2(const std::string& name, glm::vec2 &value) const;
	void setVec2(const std::string& name, float x, float y) const;
	void setVec3(const std::string& name, glm::vec3 &value) const;
	void setVec3(const std::string& name, float x, float y,float z) const;
	void setVec4(const std::string& name, glm::vec4& value) const;
	void setVec4(const std::string& name, float x, float y, float z , float w) const;
	void setMat2(const std::string& name, glm::mat2 &value) const;
	void setMat3(const std::string& name, glm::mat3 &value) const;
	void setMat4(const std::string& name, const glm::mat4 &value) const;
};