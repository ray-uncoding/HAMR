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

// Draw 類別的建構函式
Draw::Draw()
{
	// 初始化圓柱的半徑、高度和頂點數據
	float r[] = { 428.31,172.348,252.816,145.719,118.029,0 };
	float h[] = { 746.81,1152.263,571.044,861.908,334.494,0 };
	float b[] = {
		0.0f, 0.0f, 0.0f, 0.15f, 0.0f, 0.0f, 0.15f, 0.15f, 0.0f, 0.0f, 0.15f, 0.0f,
		0.0f, 0.0f, 0.15f, 0.15f, 0.0f, 0.15f, 0.15f, 0.15f, 0.15f, 0.0f, 0.15f, 0.15f,
		0.0f, 0.15f, 0.15f, 0.0f, 0.15f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.15f,
		0.15f, 0.15f, 0.15f, 0.15f, 0.15f, 0.0f, 0.15f, 0.0f, 0.0f, 0.15f, 0.0f, 0.15f,
		0.0f, 0.0f, 0.0f, 0.15f, 0.0f, 0.0f, 0.15f, 0.0f, 0.15f, 0.0f, 0.0f, 0.15f,
		0.0f, 0.15f, 0.0f, 0.15f, 0.15f, 0.0f, 0.15f, 0.15f, 0.15f, 0.0f, 0.15f, 0.15f,
	};
	GLuint i[] = {
		0, 1, 3, 1, 2, 3, 4, 5, 7, 5, 6, 7,
		8, 9, 11, 9, 10, 11, 12, 13, 15, 13, 14, 15,
		16, 17, 19, 17, 18, 19, 20, 21, 23, 21, 22, 23
	};
	GLfloat l[] = { -0.5f, -0.5f, -0.5f };

	// 將資料複製到成員變數中
	memcpy(radius, r, 6 * sizeof(float));
	memcpy(height, h, 6 * sizeof(float));
	memcpy(box, b, 72 * sizeof(float));
	memcpy(indice, i, 36 * sizeof(GLuint));
	memcpy(lineVertices, l, 3 * sizeof(GLfloat));
}

Draw::~Draw() {
	// 解構函式，用於釋放資源（若有需要）
}

// 初始化函式
void Draw::init() {
	glEnable(GL_DEPTH_TEST);  // 開啟深度測試
	program = compile_shader(vertexShaderSource, fragmentShaderSource);  // 編譯著色器

	// 設定圓柱參數並生成 VAO、VBO、EBO
	for (int i = 0; i < 6; i++) {
		cylinder[i].setHeight(height[i] / scale);  // 設定圓柱高度
		cylinder[i].setBaseRadius(radius[i] / scale);  // 設定圓柱底半徑
		cylinder[i].setTopRadius(radius[i] / scale);  // 設定圓柱頂半徑
		glGenVertexArrays(1, &VAO[i]);
		glGenBuffers(1, &VBO[i]);
		glGenBuffers(1, &EBO[i]);
		bindVertex(cylinder[i], VAO[i], VBO[i], EBO[i]);  // 綁定頂點屬性
	}

	// 初始化線段的 VAO 和 VBO
	glGenVertexArrays(1, &VAOline);
	glGenBuffers(1, &VBOline);
	glBindVertexArray(VAOline);

	glBindBuffer(GL_ARRAY_BUFFER, VBOline);
	glBufferData(GL_ARRAY_BUFFER, 1024 * sizeof(GLfloat), nullptr, GL_DYNAMIC_DRAW);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
	glEnableVertexAttribArray(0);

	glBindVertexArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
}

// 顯示物體的函式
void Draw::showObject(int pos, std::vector<float> angledeg, GLfloat* newVertices, GLFWwindow* window) {
	float angle[5];
	for (int i = 0; i < 5; i++) {
		angle[i] = glm::radians(angledeg[i]);  // 將角度轉換為弧度
	}

	// 計算時間差
	float currentFrame = static_cast<float>(glfwGetTime());
	deltaTime = currentFrame - lastFrame;
	lastFrame = currentFrame;

	processInput(window);  // 處理使用者輸入

	// 清除緩衝區，設定背景顏色
	glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glUseProgram(program);  // 使用著色器程式

	// 設定視圖矩陣
	glm::mat4 rotation = glm::mat4(1.0f);
	glm::mat4 view = glm::lookAt(cameraPos, cameraPos + cameraFront, cameraUp);
	rotation = glm::rotate(rotation, glm::radians(-90.0f), glm::vec3(1.0f, 0.0f, 0.0f));
	rotation = glm::rotate(rotation, glm::radians(-90.0f), glm::vec3(0.0f, 0.0f, 1.0f));
	//rotation = glm::rotate(rotation, glm::radians(90.0f), glm::vec3(0.0f, 0.0f, 1.0f));
	//rotation = glm::rotate(rotation, glm::radians(-90.0f), glm::vec3(0.0f, 0.0f, 1.0f));
	view = view * rotation;

	GLuint viewLoc = glGetUniformLocation(program, "view");
	glUniformMatrix4fv(viewLoc, 1, GL_FALSE, &view[0][0]);

	// 設定投影矩陣
	glm::mat4 projection = glm::mat4(1.0f);
	float fov = 100.0f; // 增大視角
	projection = glm::perspective(glm::radians(fov), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 200.0f);
	GLuint projectionLoc = glGetUniformLocation(program, "projection");
	glUniformMatrix4fv(projectionLoc, 1, GL_FALSE, &projection[0][0]);

	GLuint modelLoc = glGetUniformLocation(program, "model");
	GLint colorLocation = glGetUniformLocation(program, "ourColor");

	// link 0 的模型設定
	glm::mat4 rot = glm::mat4(1.0f);
	glm::mat4 trans = glm::mat4(1.0f);
	glm::mat4 model = glm::mat4(1.0f);

	// 設定顏色與模型變換
	glUniform4f(colorLocation, 0.0f, 1.0f, 0.0f, 1.0f);
	glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));
	draw_cylinder(cylinder[0], VAO[0]);

	// link 1 的模型設定
	glUniform4f(colorLocation, 1.0f, 0.0f, 0.0f, 1.0f);
	rot = glm::mat4(1.0f);
	trans = glm::mat4(1.0f);
	trans = glm::translate(trans, glm::vec3((172.348/2)/ scale, -428.31 / scale, 575 / scale));
	rot = glm::rotate(rot, glm::radians(90.0f), glm::vec3(0.0f, 0.0f, 1.0f)); //J1 joint1
	//rot = glm::rotate(rot, glm::radians(90.0f), glm::vec3(0.0f, 1.0f, 0.0f)); //J1 joint1
	//rot = glm::rotate(rot, glm::radians(90.0f), glm::vec3(1.0f, 0.0f, 0.0f)); //J1 joint1
	model = model * trans * rot;
	glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));
	draw_cylinder(cylinder[1], VAO[1]);

	// link 2 的模型設定
	//glBindVertexArray(VAO2);
	glUniform4f(colorLocation, 0.0f, 0.0f, 1.0f, 1.0f);
	rot = glm::mat4(1.0f);
	trans = glm::mat4(1.0f);
	trans = glm::translate(trans, glm::vec3(0.0f, 0.0f, (890+ 252.816) / scale));//trans = glm::translate(trans, glm::vec3(0.0f, -172.348/2, 575));
	//trans = glm::translate(trans, glm::vec3(0.0f, -175 / scale, 0.0f));
	trans = glm::translate(trans, glm::vec3(-(571.044 / 2 -428.31) / scale, 0.0f, 0.0f));
	rot = glm::rotate(rot, glm::radians(90.0f), glm::vec3(0.0f, 1.0f, 0.0f)); //J2
	//trans = glm::translate(trans, glm::vec3(428.31 / scale, 0.0f, 0.0f));
	//rot = glm::rotate(rot, angle[1], glm::vec3(0.0f, 0.0f, 1.0f)); //J2
	model = model * trans * rot;
	glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));
	draw_cylinder(cylinder[2], VAO[2]);

	// link 3 的模型設定
	//glBindVertexArray(VAO3);
	glUniform4f(colorLocation, 0.0f, 1.0f, 1.0f, 1.0f);
	rot = glm::mat4(1.0f);
	rot = glm::rotate(rot, glm::radians(90.0f), glm::vec3(1.0f, 0.0f, 0.0f));
	//rot = glm::rotate(rot, glm::radians(90.0f), glm::vec3(1.0f, 0.0f, 0.0f));
	trans = glm::mat4(1.0f);
	trans = glm::translate(trans, glm::vec3(0.0f, 0.0f, 252.816 / scale)); //translation (distance, axis)
	trans = glm::translate(trans, glm::vec3(0.0f, -252.816 / scale, 0.0f)); //translation (distance, axis)
	rot = glm::rotate(rot, glm::radians(90.0f), glm::vec3(0.0f, 0.0f, 1.0f)); //rotation (angle, axis)
	//rot = glm::rotate(rot, glm::radians(90.0f), glm::vec3(0.0f, 0.0f, 1.0f));
	//rot = glm::rotate(rot, glm::radians(90.0f), glm::vec3(1.0f, 0.0f, 0.0f)); //J3
	model = model * trans * rot;
	glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));
	draw_cylinder(cylinder[3], VAO[3]);

	// link 4 的模型設定
	//glBindVertexArray(VAO4);
	glUniform4f(colorLocation, 1.0f, 0.0f, 1.0f, 1.0f);
	rot = glm::mat4(1.0f);
	trans = glm::mat4(1.0f);
	trans = glm::translate(trans, glm::vec3(0.0f, 0.0f, 861.908 / scale));
	trans = glm::translate(trans, glm::vec3(-(334.494/2) / scale, 0.0f, 0.0f));
	rot = glm::rotate(rot, glm::radians(90.0f), glm::vec3(0.0f, 1.0f, 0.0f)); //J4
	model = model * trans * rot;
	glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));
	draw_cylinder(cylinder[4], VAO[4]);

	// link 5 的模型設定
	//glBindVertexArray(VAO5);
	glUniform4f(colorLocation, 1.0f, 1.0f, 0.0f, 1.0f);
	rot = glm::mat4(1.0f);
	trans = glm::mat4(1.0f);
	trans = glm::translate(trans, glm::vec3(0.0f, 0.0f, 349.0 / scale));
	rot = glm::rotate(rot, angle[4], glm::vec3(1.0f, 0.0f, 0.0f));//J5
	model = model * trans * rot;
	
	glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));
	draw_cylinder(cylinder[5], VAO[5]);
	
	glm::mat4 boxPos(5.0f); // 初始化矩陣，用於設定box的位置（此處為縮放矩陣）
	// boxPos = translation(0 * 150, X) * translation(0 * 150, Y) * translation(0 * 150, Z); // 偏移
	// boxPos = boxPos* translation(6 * 150, X) * translation(10 * 150, Y) * translation(3 * 150, Z); // X-Y-Z方向的偏移量
	glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(boxPos)); // 設定box的位置
	glUniform4f(colorLocation, 1.0f, 1.0f, 1.0f, 1.0f); // 設定box顏色為白色

	glBindBuffer(GL_ARRAY_BUFFER, VBOline); // 綁定線段的頂點緩衝

	// 更新頂點緩衝數據，用於繪製新頂點
	glBufferSubData(GL_ARRAY_BUFFER, pos * sizeof(GLfloat) * 3, sizeof(GLfloat) * 3, newVertices);
	glBindVertexArray(VAOline); // 綁定線段的VAO
	glPointSize(10.0f); // 設定點的大小為10
	glDrawArrays(GL_POINTS, 0, pos * sizeof(GLfloat) * 3); // 繪製點
	glBindVertexArray(0); // 解綁VAO
	glfwSwapBuffers(window); // 交換緩衝區，顯示內容
	glfwPollEvents(); // 處理所有已觸發的事件
}
GLuint Draw::compile_shader(const char* vertex, const char* fragment) {
	GLuint vertex_shader, fragment_shader;

	//vertex shader創建頂點著色器
	vertex_shader = glCreateShader(GL_VERTEX_SHADER);
	glShaderSource(vertex_shader, 1, &vertex, NULL);
	glCompileShader(vertex_shader);
	// check for shader compile errors檢查頂點著色器編譯錯誤
	int success;
	char infoLog[512];
	glGetShaderiv(vertex_shader, GL_COMPILE_STATUS, &success);
	if (!success)
	{
		glGetShaderInfoLog(vertex_shader, 512, NULL, infoLog);
		std::cout << "ERROR::SHADER::VERTEX::COMPILATION_FAILED\n" << infoLog << std::endl;
	}

	// fragment shader創建片段著色器
	fragment_shader = glCreateShader(GL_FRAGMENT_SHADER);
	glShaderSource(fragment_shader, 1, &fragment, NULL);
	glCompileShader(fragment_shader);
	// check for shader compile errors檢查片段著色器編譯錯誤
	glGetShaderiv(fragment_shader, GL_COMPILE_STATUS, &success);
	if (!success)
	{
		glGetShaderInfoLog(fragment_shader, 512, NULL, infoLog);
		std::cout << "ERROR::SHADER::FRAGMENT::COMPILATION_FAILED\n" << infoLog << std::endl;
	}

	// link shaders 連結著色器
	GLuint shaderProgram = glCreateProgram();
	glAttachShader(shaderProgram, vertex_shader);
	glAttachShader(shaderProgram, fragment_shader);
	glLinkProgram(shaderProgram);
	// check for linking errors 檢查著色器連結錯誤
	glGetProgramiv(shaderProgram, GL_LINK_STATUS, &success);
	if (!success) {
		glGetProgramInfoLog(shaderProgram, 512, NULL, infoLog);
		std::cout << "ERROR::SHADER::PROGRAM::LINKING_FAILED\n" << infoLog << std::endl;
	}
	glDeleteShader(vertex_shader); // 刪除頂點著色器
	glDeleteShader(fragment_shader); // 刪除片段著色器
	std::cout << "shader compiled" << std::endl;
	return shaderProgram;
}
void Draw::bindVertex(Cylinder cylinder, GLuint VAO, GLuint VBO, GLuint EBO) {
	// bind the Vertex Array Object first, then bind and set vertex buffer(s), and then configure vertex attributes(s).
	// 綁定VAO，接著綁定並設置頂點緩衝，再設置頂點屬性
	glBindVertexArray(VAO);

	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferData(GL_ARRAY_BUFFER, cylinder.getVertexSize(), cylinder.getVertices(), GL_DYNAMIC_DRAW);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, cylinder.getIndexSize(), cylinder.getIndices(), GL_STATIC_DRAW);

	// 設置頂點屬性指針
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);
}
glm::mat4 Draw::rotation(float angle, Direction axis) {
	glm::mat4 rot = glm::mat4(1.0f); // 初始化旋轉矩陣
	switch (axis) {
	case X:
		// 繞X軸旋轉的矩陣
		rot = glm::mat4(1.0f, 0.0f, 0.0f, 0.0f,
			0.0f, cos(glm::radians(angle)), sin(glm::radians(angle)), 0.0f,
			0.0f, -sin(glm::radians(angle)), cos(glm::radians(angle)), 0.0f,
			0.0f, 0.0f, 0.0f, 1.0f);
		break;

	case Y:
		// 繞Y軸旋轉的矩陣
		rot = glm::mat4(cos(glm::radians(angle)), 0.0f, -sin(glm::radians(angle)), 0.0f,
			0.0f, 1.0f, 0.0f, 0.0f,
			sin(glm::radians(angle)), 0.0f, cos(glm::radians(angle)), 0.0f,
			0.0f, 0.0f, 0.0f, 1.0f);
		break;

	case Z:
		// 繞Z軸旋轉的矩陣
		rot = glm::mat4(cos(glm::radians(angle)), sin(glm::radians(angle)), 0.0f, 0.0f,
			-sin(glm::radians(angle)), cos(glm::radians(angle)), 0.0f, 0.0f,
			0.0f, 0.0f, 1.0f, 0.0f,
			0.0f, 0.0f, 0.0f, 1.0f);
		break;
	}
	return rot;
}

glm::mat4 Draw::translation(float distance, Direction axis) {
	glm::mat4 trans = glm::mat4(1.0f); // 初始化平移矩陣
	switch (axis) {
	case X:
		// 沿X軸方向的平移
		trans = glm::mat4(1.0f, 0.0f, 0.0f, 0.0f,
			0.0f, 1.0f, 0.0f, 0.0f,
			0.0f, 0.0f, 1.0f, 0.0f,
			distance / scale, 0.0f, 0.0f, 1.0f);
		break;

	case Y:
		// 沿Y軸方向的平移
		trans = glm::mat4(1.0f, 0.0f, 0.0f, 0.0f,
			0.0f, 1.0f, 0.0f, 0.0f,
			0.0f, 0.0f, 1.0f, 0.0f,
			0.0f, distance / scale, 0.0f, 1.0f);
		break;

	case Z:
		// 沿Z軸方向的平移
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
	glBindVertexArray(vertex); // 綁定頂點數據
	glDrawElements(GL_TRIANGLES, cylinder.getIndexCount(), GL_UNSIGNED_INT, 0); // 繪製三角形元素
	glBindVertexArray(0); // 解綁VAO
}
void Draw::processInput(GLFWwindow* window)
{
	const float cameraSpeed = static_cast<float>(2.5 * deltaTime); // 相機移動速度
	if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
		cameraPos -= cameraSpeed * cameraUp; // 向上移動
	if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
		cameraPos += cameraSpeed * cameraUp; // 向下移動
	if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
		cameraPos += glm::normalize(glm::cross(cameraFront, cameraUp)) * cameraSpeed; // 向左移動
	if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
		cameraPos -= glm::normalize(glm::cross(cameraFront, cameraUp)) * cameraSpeed; // 向右移動
}
void Draw::terminate() {
	glDeleteVertexArrays(1, VAO);
	glDeleteBuffers(1, VBO);
	glDeleteVertexArrays(1, &VAOline);
	glDeleteBuffers(1, &VBOline);
	glDeleteBuffers(1, EBO);
	glDeleteProgram(program);
	glfwTerminate();
}