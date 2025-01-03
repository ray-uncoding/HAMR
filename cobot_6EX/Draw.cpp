#include <iostream>
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <math.h>
#include "shader.h"
#include "cylinder/cylinder/src/Cylinder.h"
#include "Draw.h"
#include <glm/gtx/string_cast.hpp>

Draw::Draw()
{

	float r[] = {428.31, 172.348, 252.816, 145.719, 118.029, 0};
	float h[] = {746.81, 1152.263, 571.044, 861.908, 334.494, 0};
	float b[] = {
		0.0f,
		0.0f,
		0.0f,
		0.15f,
		0.0f,
		0.0f,
		0.15f,
		0.15f,
		0.0f,
		0.0f,
		0.15f,
		0.0f,
		0.0f,
		0.0f,
		0.15f,
		0.15f,
		0.0f,
		0.15f,
		0.15f,
		0.15f,
		0.15f,
		0.0f,
		0.15f,
		0.15f,
		0.0f,
		0.15f,
		0.15f,
		0.0f,
		0.15f,
		0.0f,
		0.0f,
		0.0f,
		0.0f,
		0.0f,
		0.0f,
		0.15f,
		0.15f,
		0.15f,
		0.15f,
		0.15f,
		0.15f,
		0.0f,
		0.15f,
		0.0f,
		0.0f,
		0.15f,
		0.0f,
		0.15f,
		0.0f,
		0.0f,
		0.0f,
		0.15f,
		0.0f,
		0.0f,
		0.15f,
		0.0f,
		0.15f,
		0.0f,
		0.0f,
		0.15f,
		0.0f,
		0.15f,
		0.0f,
		0.15f,
		0.15f,
		0.0f,
		0.15f,
		0.15f,
		0.15f,
		0.0f,
		0.15f,
		0.15f,
	};
	GLuint i[] = {
		0, 1, 3, 1, 2, 3, 4, 5, 7, 5, 6, 7,
		8, 9, 11, 9, 10, 11, 12, 13, 15, 13, 14, 15,
		16, 17, 19, 17, 18, 19, 20, 21, 23, 21, 22, 23};
	GLfloat l[] = {-0.5f, -0.5f, -0.5f};

	memcpy(radius, r, 6 * sizeof(float));
	memcpy(height, h, 6 * sizeof(float));
	memcpy(box, b, 72 * sizeof(float));
	memcpy(indice, i, 36 * sizeof(GLuint));
	memcpy(lineVertices, l, 3 * sizeof(GLfloat));
}

Draw::~Draw()
{
}

void Draw::init()
{
	glEnable(GL_DEPTH_TEST);
	program = compile_shader(vertexShaderSource, fragmentShaderSource);

	// �]�w��W�Ѽƨåͦ� VAO�BVBO�BEBO
	for (int i = 0; i < 6; i++)
	{
		cylinder[i].setHeight(height[i] / scale);
		cylinder[i].setBaseRadius(radius[i] / scale);
		cylinder[i].setTopRadius(radius[i] / scale);
		glGenVertexArrays(1, &VAO[i]);
		glGenBuffers(1, &VBO[i]);
		glGenBuffers(1, &EBO[i]);
		bindVertex(cylinder[i], VAO[i], VBO[i], EBO[i]);
	}

	glGenVertexArrays(1, &VAOline);
	glGenBuffers(1, &VBOline);
	glBindVertexArray(VAOline);

	glBindBuffer(GL_ARRAY_BUFFER, VBOline);
	glBufferData(GL_ARRAY_BUFFER, 1024 * sizeof(GLfloat), nullptr, GL_DYNAMIC_DRAW);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void *)0);
	glEnableVertexAttribArray(0);

	glBindVertexArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void Draw::showObject(int pos, std::vector<float> angledeg, GLfloat *newVertices, GLFWwindow *window)
{
	float angle[5];
	for (int i = 0; i < 5; i++)
	{
		angle[i] = glm::radians(angledeg[i]);
	}

	float currentFrame = static_cast<float>(glfwGetTime());
	deltaTime = currentFrame - lastFrame;
	lastFrame = currentFrame;

	processInput(window);

	glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glUseProgram(program);

	glm::mat4 rotation = glm::mat4(1.0f);
	glm::mat4 view = glm::lookAt(cameraPos, cameraPos + cameraFront, cameraUp);
	rotation = glm::rotate(rotation, glm::radians(-90.0f), glm::vec3(1.0f, 0.0f, 0.0f));
	rotation = glm::rotate(rotation, glm::radians(-90.0f), glm::vec3(0.0f, 0.0f, 1.0f));
	view = view * rotation;

	GLuint viewLoc = glGetUniformLocation(program, "view");
	glUniformMatrix4fv(viewLoc, 1, GL_FALSE, &view[0][0]);
	glm::mat4 projection = glm::mat4(1.0f);
	float fov = 100.0f;
	projection = glm::perspective(glm::radians(fov), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 200.0f);
	GLuint projectionLoc = glGetUniformLocation(program, "projection");
	glUniformMatrix4fv(projectionLoc, 1, GL_FALSE, &projection[0][0]);

	GLuint modelLoc = glGetUniformLocation(program, "model");
	GLint colorLocation = glGetUniformLocation(program, "ourColor");

	glm::mat4 rot = glm::mat4(1.0f);
	glm::mat4 trans = glm::mat4(1.0f);
	glm::mat4 model = glm::mat4(1.0f);
	glUniform4f(colorLocation, 0.0f, 1.0f, 0.0f, 1.0f);
	glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));
	draw_cylinder(cylinder[0], VAO[0]);

	glUniform4f(colorLocation, 1.0f, 0.0f, 0.0f, 1.0f);
	rot = glm::mat4(1.0f);
	trans = glm::mat4(1.0f);
	trans = glm::translate(trans, glm::vec3((172.348 / 2) / scale, -428.31 / scale, 575 / scale));
	rot = glm::rotate(rot, glm::radians(90.0f), glm::vec3(0.0f, 0.0f, 1.0f));
	model = model * trans * rot;
	glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));
	draw_cylinder(cylinder[1], VAO[1]);

	glUniform4f(colorLocation, 0.0f, 0.0f, 1.0f, 1.0f);
	rot = glm::mat4(1.0f);
	trans = glm::mat4(1.0f);
	trans = glm::translate(trans, glm::vec3(0.0f, 0.0f, (890 + 252.816) / scale));
	// trans = glm::translate(trans, glm::vec3(0.0f, -175 / scale, 0.0f));
	trans = glm::translate(trans, glm::vec3(-(571.044 / 2 - 428.31) / scale, 0.0f, 0.0f));
	rot = glm::rotate(rot, glm::radians(90.0f), glm::vec3(0.0f, 1.0f, 0.0f)); // J2
	// trans = glm::translate(trans, glm::vec3(428.31 / scale, 0.0f, 0.0f));
	// rot = glm::rotate(rot, angle[1], glm::vec3(0.0f, 0.0f, 1.0f)); //J2
	model = model * trans * rot;
	glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));
	draw_cylinder(cylinder[2], VAO[2]);

	// link 3 ���ҫ��]�w
	// glBindVertexArray(VAO3);
	glUniform4f(colorLocation, 0.0f, 1.0f, 1.0f, 1.0f);
	rot = glm::mat4(1.0f);
	rot = glm::rotate(rot, glm::radians(90.0f), glm::vec3(1.0f, 0.0f, 0.0f));
	// rot = glm::rotate(rot, glm::radians(90.0f), glm::vec3(1.0f, 0.0f, 0.0f));
	trans = glm::mat4(1.0f);
	trans = glm::translate(trans, glm::vec3(0.0f, 0.0f, 252.816 / scale));	  // translation (distance, axis)
	trans = glm::translate(trans, glm::vec3(0.0f, -252.816 / scale, 0.0f));	  // translation (distance, axis)
	rot = glm::rotate(rot, glm::radians(90.0f), glm::vec3(0.0f, 0.0f, 1.0f)); // rotation (angle, axis)
	// rot = glm::rotate(rot, glm::radians(90.0f), glm::vec3(0.0f, 0.0f, 1.0f));
	// rot = glm::rotate(rot, glm::radians(90.0f), glm::vec3(1.0f, 0.0f, 0.0f)); //J3
	model = model * trans * rot;
	glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));
	draw_cylinder(cylinder[3], VAO[3]);

	// link 4 ���ҫ��]�w
	// glBindVertexArray(VAO4);
	glUniform4f(colorLocation, 1.0f, 0.0f, 1.0f, 1.0f);
	rot = glm::mat4(1.0f);
	trans = glm::mat4(1.0f);
	trans = glm::translate(trans, glm::vec3(0.0f, 0.0f, 861.908 / scale));
	trans = glm::translate(trans, glm::vec3(-(334.494 / 2) / scale, 0.0f, 0.0f));
	rot = glm::rotate(rot, glm::radians(90.0f), glm::vec3(0.0f, 1.0f, 0.0f)); // J4
	model = model * trans * rot;
	glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));
	draw_cylinder(cylinder[4], VAO[4]);

	// link 5 ���ҫ��]�w
	// glBindVertexArray(VAO5);
	glUniform4f(colorLocation, 1.0f, 1.0f, 0.0f, 1.0f);
	rot = glm::mat4(1.0f);
	trans = glm::mat4(1.0f);
	trans = glm::translate(trans, glm::vec3(0.0f, 0.0f, 349.0 / scale));
	rot = glm::rotate(rot, angle[4], glm::vec3(1.0f, 0.0f, 0.0f)); // J5
	model = model * trans * rot;

	glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));
	draw_cylinder(cylinder[5], VAO[5]);

	glm::mat4 boxPos(5.0f); // ��l�Ưx�}�A�Ω�]�wbox����m�]���B���Y��x�}�^
	// boxPos = translation(0 * 150, X) * translation(0 * 150, Y) * translation(0 * 150, Z); // ����
	// boxPos = boxPos* translation(6 * 150, X) * translation(10 * 150, Y) * translation(3 * 150, Z); // X-Y-Z��V�������q
	glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(boxPos)); // �]�wbox����m
	glUniform4f(colorLocation, 1.0f, 1.0f, 1.0f, 1.0f);				   // �]�wbox�C�⬰�զ�

	glBindBuffer(GL_ARRAY_BUFFER, VBOline); // �j�w�u�q�����I�w��

	// ��s���I�w�ļƾڡA�Ω�ø�s�s���I
	glBufferSubData(GL_ARRAY_BUFFER, pos * sizeof(GLfloat) * 3, sizeof(GLfloat) * 3, newVertices);
	glBindVertexArray(VAOline);							   // �j�w�u�q��VAO
	glPointSize(10.0f);									   // �]�w�I���j�p��10
	glDrawArrays(GL_POINTS, 0, pos * sizeof(GLfloat) * 3); // ø�s�I
	glBindVertexArray(0);								   // �ѸjVAO
	glfwSwapBuffers(window);							   // �洫�w�İϡA��ܤ��e
	glfwPollEvents();									   // �B�z�Ҧ��wĲ�o���ƥ�
}
GLuint Draw::compile_shader(const char *vertex, const char *fragment)
{
	GLuint vertex_shader, fragment_shader;

	// vertex shader�Ыس��I�ۦ⾹
	vertex_shader = glCreateShader(GL_VERTEX_SHADER);
	glShaderSource(vertex_shader, 1, &vertex, NULL);
	glCompileShader(vertex_shader);
	// check for shader compile errors�ˬd���I�ۦ⾹�sĶ���~
	int success;
	char infoLog[512];
	glGetShaderiv(vertex_shader, GL_COMPILE_STATUS, &success);
	if (!success)
	{
		glGetShaderInfoLog(vertex_shader, 512, NULL, infoLog);
		std::cout << "ERROR::SHADER::VERTEX::COMPILATION_FAILED\n"
				  << infoLog << std::endl;
	}

	// fragment shader�Ыؤ��q�ۦ⾹
	fragment_shader = glCreateShader(GL_FRAGMENT_SHADER);
	glShaderSource(fragment_shader, 1, &fragment, NULL);
	glCompileShader(fragment_shader);
	// check for shader compile errors�ˬd���q�ۦ⾹�sĶ���~
	glGetShaderiv(fragment_shader, GL_COMPILE_STATUS, &success);
	if (!success)
	{
		glGetShaderInfoLog(fragment_shader, 512, NULL, infoLog);
		std::cout << "ERROR::SHADER::FRAGMENT::COMPILATION_FAILED\n"
				  << infoLog << std::endl;
	}

	// link shaders �s���ۦ⾹
	GLuint shaderProgram = glCreateProgram();
	glAttachShader(shaderProgram, vertex_shader);
	glAttachShader(shaderProgram, fragment_shader);
	glLinkProgram(shaderProgram);
	// check for linking errors �ˬd�ۦ⾹�s�����~
	glGetProgramiv(shaderProgram, GL_LINK_STATUS, &success);
	if (!success)
	{
		glGetProgramInfoLog(shaderProgram, 512, NULL, infoLog);
		std::cout << "ERROR::SHADER::PROGRAM::LINKING_FAILED\n"
				  << infoLog << std::endl;
	}
	glDeleteShader(vertex_shader);	 // �R�����I�ۦ⾹
	glDeleteShader(fragment_shader); // �R�����q�ۦ⾹
	std::cout << "shader compiled" << std::endl;
	return shaderProgram;
}
void Draw::bindVertex(Cylinder cylinder, GLuint VAO, GLuint VBO, GLuint EBO)
{
	// bind the Vertex Array Object first, then bind and set vertex buffer(s), and then configure vertex attributes(s).
	// �j�wVAO�A���۸j�w�ó]�m���I�w�ġA�A�]�m���I�ݩ�
	glBindVertexArray(VAO);

	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferData(GL_ARRAY_BUFFER, cylinder.getVertexSize(), cylinder.getVertices(), GL_DYNAMIC_DRAW);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, cylinder.getIndexSize(), cylinder.getIndices(), GL_STATIC_DRAW);

	// �]�m���I�ݩʫ��w
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void *)0);
	glEnableVertexAttribArray(0);
}
glm::mat4 Draw::rotation(float angle, Direction axis)
{
	glm::mat4 rot = glm::mat4(1.0f); // ��l�Ʊ���x�}
	switch (axis)
	{
	case X:
		// ¶X�b���઺�x�}
		rot = glm::mat4(1.0f, 0.0f, 0.0f, 0.0f,
						0.0f, cos(glm::radians(angle)), sin(glm::radians(angle)), 0.0f,
						0.0f, -sin(glm::radians(angle)), cos(glm::radians(angle)), 0.0f,
						0.0f, 0.0f, 0.0f, 1.0f);
		break;

	case Y:
		// ¶Y�b���઺�x�}
		rot = glm::mat4(cos(glm::radians(angle)), 0.0f, -sin(glm::radians(angle)), 0.0f,
						0.0f, 1.0f, 0.0f, 0.0f,
						sin(glm::radians(angle)), 0.0f, cos(glm::radians(angle)), 0.0f,
						0.0f, 0.0f, 0.0f, 1.0f);
		break;

	case Z:
		// ¶Z�b���઺�x�}
		rot = glm::mat4(cos(glm::radians(angle)), sin(glm::radians(angle)), 0.0f, 0.0f,
						-sin(glm::radians(angle)), cos(glm::radians(angle)), 0.0f, 0.0f,
						0.0f, 0.0f, 1.0f, 0.0f,
						0.0f, 0.0f, 0.0f, 1.0f);
		break;
	}
	return rot;
}

glm::mat4 Draw::translation(float distance, Direction axis)
{
	glm::mat4 trans = glm::mat4(1.0f); // ��l�ƥ����x�}
	switch (axis)
	{
	case X:
		// �uX�b��V������
		trans = glm::mat4(1.0f, 0.0f, 0.0f, 0.0f,
						  0.0f, 1.0f, 0.0f, 0.0f,
						  0.0f, 0.0f, 1.0f, 0.0f,
						  distance / scale, 0.0f, 0.0f, 1.0f);
		break;

	case Y:
		// �uY�b��V������
		trans = glm::mat4(1.0f, 0.0f, 0.0f, 0.0f,
						  0.0f, 1.0f, 0.0f, 0.0f,
						  0.0f, 0.0f, 1.0f, 0.0f,
						  0.0f, distance / scale, 0.0f, 1.0f);
		break;

	case Z:
		// �uZ�b��V������
		trans = glm::mat4(1.0f, 0.0f, 0.0f, 0.0f,
						  0.0f, 1.0f, 0.0f, 0.0f,
						  0.0f, 0.0f, 1.0f, 0.0f,
						  0.0f, 0.0f, distance / scale, 1.0f);
		break;
	}
	return trans;
}

void Draw::draw_cylinder(Cylinder cylinder, GLuint vertex)
{
	glBindVertexArray(vertex);													
	glDrawElements(GL_TRIANGLES, cylinder.getIndexCount(), GL_UNSIGNED_INT, 0); 
	glBindVertexArray(0);														
}
void Draw::processInput(GLFWwindow *window)
{
	const float cameraSpeed = static_cast<float>(2.5 * deltaTime);
	if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
		cameraPos -= cameraSpeed * cameraUp; 
	if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
		cameraPos += cameraSpeed * cameraUp; 
	if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
		cameraPos += glm::normalize(glm::cross(cameraFront, cameraUp)) * cameraSpeed; 
	if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
		cameraPos -= glm::normalize(glm::cross(cameraFront, cameraUp)) * cameraSpeed; 
}
void Draw::terminate()
{
	glDeleteVertexArrays(1, VAO);
	glDeleteBuffers(1, VBO);
	glDeleteVertexArrays(1, &VAOline);
	glDeleteBuffers(1, &VBOline);
	glDeleteBuffers(1, EBO);
	glDeleteProgram(program);
	glfwTerminate();
}