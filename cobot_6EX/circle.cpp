//#include <glad/glad.h>
//#include <GLFW/glfw3.h>
//#include <math.h>
//#include <iostream>
//
//const char* vertexShaderSource = "#version 120\n"
//"attribute vec3 inColor;\n"
//"attribute vec3 inPosition;"
//"uniform mat4 matrix;"
//"varying vec3 outColor;" 
//"void main()\n"
//"{\n"
//"  outColor = inColor;\n"
//"  gl_Position = vec4(inPosition,1);\n"
//"}\0";
//const char* fragmentShaderSource = "#version 120\n"
//"varying vec3 outColor;\n"
//"void main()\n"
//"{\n"
//"   gl_FragColor = vec4(outColor,1);\n"
//"}\n\0";
//
//const GLfloat position[]={
//	-1.0f, -1.0f, 0.0f,
//	 1.0f, -1.0f, 0.0f,
//	 1.0f,  1.0f, 0.0f,
//	-1.0f, -1.0f, 0.0f,
//	 1.0f,  1.0f, 0.0f,
//	-1.0f,  1.0f, 0.0f
//};
//
//const GLfloat color[] = {
//	0.0f, 0.0f, 1.0f,
//	0.0f, 1.0f, 0.0f,
//	1.0f, 0.0f, 0.0f,
//	0.0f, 0.0f, 1.0f,
//	1.0f, 0.0f, 0.0f,
//	0.0f, 1.0f, 0.0f
//};
//
//void drawCircle(float r, float g, float b) {
//	const int steps = 50;
//	const float angle = 3.1415926 * 2.f / steps;
//	float xPos = 0, yPos = 0, radius = 1.0f;
//	float prevX = xPos;
//	float prevY = yPos - radius;
//
//	for (int i = 0; i <= steps; i++)
//	{
//		float newX = radius * sin(angle * i);
//		float newY = -radius * cos(angle * i);
//
//		glBegin(GL_TRIANGLES);
//		glColor3f(r, g, b);
//		glVertex3f(0.0f, 0.0f, 0.0f);
//		glVertex3f(prevX, prevY, 0.0f);
//		glVertex3f(newX, newY, 0.0f);
//		glEnd();
//
//		prevX = newX;
//		prevY = newY;
//	}
//}
//
//int main() {
//	GLFWwindow* window;
//
//	if (!glfwInit()) {
//		std::cout << "Init error";
//		return -1;
//	}
//
//	window = glfwCreateWindow(800, 800, "Circle", 0, 0);
//	if (!window) {
//		std::cout << "Window creation error";
//		glfwTerminate();
//		return -1;
//	}
//
//	glfwMakeContextCurrent(window);
//	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
//	{
//		std::cout << "Failed to initialize GLAD" << std::endl;
//		return -1;
//	}
//	unsigned int  vertexShader = glCreateShader(GL_VERTEX_SHADER);
//	glShaderSource(vertexShader, 1, &vertexShaderSource, 0);
//	glCompileShader(vertexShader);
//	// check for shader compile errors
//	int success;
//	char infoLog[512];
//	glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &success);
//	if (!success)
//	{
//		glGetShaderInfoLog(vertexShader, 512, NULL, infoLog);
//		std::cout << "ERROR::SHADER::VERTEX::COMPILATION_FAILED\n" << infoLog << std::endl;
//	}
//
//	// fragment shader
//	GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
//	glShaderSource(fragmentShader, 1, &fragmentShaderSource, NULL);
//	glCompileShader(fragmentShader);
//	// check for shader compile errors
//	glGetShaderiv(fragmentShader, GL_COMPILE_STATUS, &success);
//	if (!success)
//	{
//		glGetShaderInfoLog(fragmentShader, 512, NULL, infoLog);
//		std::cout << "ERROR::SHADER::FRAGMENT::COMPILATION_FAILED\n" << infoLog << std::endl;
//	}
//
//	// link shaders
//	unsigned int shaderProgram = glCreateProgram();
//	glAttachShader(shaderProgram, vertexShader);
//	glAttachShader(shaderProgram, fragmentShader);
//	glLinkProgram(shaderProgram);
//	// check for linking errors
//	glGetProgramiv(shaderProgram, GL_LINK_STATUS, &success);
//	if (!success) {
//		glGetProgramInfoLog(shaderProgram, 512, NULL, infoLog);
//		std::cout << "ERROR::SHADER::PROGRAM::LINKING_FAILED\n" << infoLog << std::endl;
//	}
//	glDeleteShader(vertexShader);
//	glDeleteShader(fragmentShader);
//	glUseProgram(shaderProgram);
//
//	GLuint positionData;
//	glGenBuffers(1, &positionData);
//	glBindBuffer(GL_ARRAY_BUFFER, positionData);
//	glBufferData(GL_ARRAY_BUFFER, sizeof(position), position, GL_STATIC_DRAW);
//	GLuint colorData;
//	glGenBuffers(1, &colorData);
//	glBindBuffer(GL_ARRAY_BUFFER, colorData);
//	glBufferData(GL_ARRAY_BUFFER, sizeof(color), color, GL_STATIC_DRAW);
//
//	GLuint attribPosition, attribColor;
//	attribPosition = glGetAttribLocation(shaderProgram, "inPosition");
//	glEnableVertexAttribArray(attribPosition);
//	glBindBuffer(GL_ARRAY_BUFFER, positionData);
//	glVertexAttribPointer(attribPosition, 3, GL_FLOAT, GL_FALSE, 0, position);
//
//	attribColor = glGetAttribLocation(shaderProgram, "inColor");
//	glEnableVertexAttribArray(attribColor);
//	glBindBuffer(GL_ARRAY_BUFFER, colorData);
//	glVertexAttribPointer(attribColor, 3, GL_FLOAT, GL_FALSE, 0, 0);
//
//	//GLuint vertexBuffer;
//	//glGenBuffers(1, &vertexBuffer);
//	//GLuint colorBuffer;
//	//glGenBuffers(1, &colorBuffer);
//
//	//glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
//	//glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
//	//glBindBuffer(GL_ARRAY_BUFFER, colorBuffer);
//	//glBufferData(GL_ARRAY_BUFFER, sizeof(color), color, GL_STATIC_DRAW);
//
//	//GLint attribPosition = glGetAttribLocation(shaderProgram, "inPosition");
//	//glEnableVertexAttribArray(attribPosition);
//	//glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
//	//glVertexAttribPointer(attribPosition, 3, GL_FLOAT, GL_FALSE, 0, 0);
//
//	//GLint attribColor = glGetAttribLocation(shaderProgram, "inColor");
//	//glEnableVertexAttribArray(attribColor);
//	//glBindBuffer(GL_ARRAY_BUFFER, colorBuffer);
//	//glVertexAttribPointer(attribColor, 3, GL_FLOAT, GL_FALSE, 0, 0);
//
//	//glMatrixMode(GL_MODELVIEW);
//	//glLoadIdentity();
//	//glScalef(0.1, 0.1, 1);
//
//	//float angle = 0;
//	//render loop
//	while (!glfwWindowShouldClose(window))
//	{
//		glClearColor(1.0, 1.0, 1.0, 0);
//		glClear(GL_COLOR_BUFFER_BIT);
//		//angle += 1;
//		//drawCircle(0.0, 1.0, 0.0);
//		//{
//		//	glPushMatrix();
//		//	glRotatef(angle, 0, 0, 1);
//		//	glTranslatef(0, 5, 0);
//		//	drawCircle(1.0, 0.0, 0.0);
//		//	glPopMatrix();
//		//}
//		glDrawArrays(GL_TRIANGLES, 0, 6);
//		glfwSwapBuffers(window);
//		glfwPollEvents();
//	}
//	glfwTerminate();
//}