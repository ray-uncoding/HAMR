//這個是繪圖的主檔案，他會調用路徑畫算法並規劃程式碼路徑

#pragma once
#include <iostream>
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <math.h>
#include "shader.h"
#include "cylinder/cylinder/src/Cylinder.h"

class Draw {
public:
	Draw();
	~Draw();
	GLFWwindow* window;
	void init();
	void showObject(int pos, std::vector<float> angle, GLfloat* newVertices, GLFWwindow* window);
	void terminate();

private:
	const char* vertexShaderSource = "#version 450 core\n"
		"layout (location = 0) in vec3 aPos;\n"
		"//layout (location = 1) in vec4 color;\n"\
		"//out vec4 TextColor;\n"
		"uniform mat4 model;\n"
		"uniform mat4 view;\n"
		"uniform mat4 projection;\n"
		"uniform mat4 gRotation;\n"
		"void main()	\n"
		"{	\n"
		"	gl_Position = projection * view * model * vec4(aPos, 1.0f);\n"
		"	//TexColor = color;\n"
		"}\0";
	const char* fragmentShaderSource = "#version 450 core\n"
		"out vec4 FragColor;\n"
		"//in vec2 TextCoord;\n"
		"uniform vec4 ourColor;\n"
		"void main()\n"
		"{\n"
		"   FragColor = ourColor;\n"
		"}\n\0";

	enum Direction {
		X = 1, Y = 2, Z = 3
	};
	const unsigned int SCR_WIDTH = 800;
	const unsigned int SCR_HEIGHT = 600;

	glm::vec3 cameraPos = glm::vec3(0.0f, 0.5f, 3.0f);
	glm::vec3 cameraFront = glm::vec3(0.0f, 0.0f, -1.0f);
	glm::vec3 cameraUp = glm::vec3(0.0f, 1.0f, 0.0f);

	bool firstMouse = true;
	float yaw = -90.0f;
	float pitch = 0.0f;
	float lastX = 800.0f / 2.0;
	float lastY = 600.0 / 2.0;
	float fov = 45.0f;
	float scale = 1000.0;
	float radius[6];
	float height[6];
	float box[72];
	GLuint indice[36];
	GLfloat lineVertices[3];

	float deltaTime;
	float lastFrame;
	Cylinder cylinder[6];
	GLuint VAO[6];
	GLuint VBO[6];
	GLuint EBO[6];
	GLuint VBOline;
	GLuint VAOline;
	GLuint program;
	GLuint compile_shader(const char* vertex, const char* fragment);
	void bindVertex(Cylinder cylinder, GLuint VAO, GLuint VBO, GLuint EBO);
	glm::mat4 rotation(float angle, Direction axis);
	glm::mat4 translation(float distance, Direction axis);
	void draw_cylinder(Cylinder cylinder, GLuint vertex);
	void processInput(GLFWwindow* window);
};
