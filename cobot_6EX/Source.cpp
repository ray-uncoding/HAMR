
#include <iostream>
#include "test.h"
#include <stdlib.h>
#include <stdio.h>
#include <windows.h>
#include <opencv2/opencv.hpp>
#include "NewtonUpdate.h"
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <math.h>
#include "shader.h"
#include "cylinder/cylinder/src/Cylinder.h"
#include "Draw.h"
//#include "Function.h"

using namespace std;
using namespace cv;
cv::Mat pathcreate(cv::Mat start, cv::Mat end, int sampleRange) {
	cv::Mat P1 = start;
	cv::Mat P2 = end;

	cv::Mat dis;

	for (int i = 0; i < P1.cols; i++) {
		dis.push_back(P1.col(i) - P2.col(i));
		//std::cout << P1.col(i) - P2.col(i) << std::endl;

	}

	cv::Mat step;
	for (int i = 0; i < P1.cols; i++) {
		step.push_back((P1.col(i) - P2.col(i)) / sampleRange);
		//std::cout << step << std::endl;
	}
	cv::Mat path;
	cv::Mat pos = (cv::Mat_<float>(1, 6) << 0, 0, 0, 0, 0, 0);
	path.push_back(P2);
	for (int i = 0; i < sampleRange; i++) {

		for (int j = 0; j < 6; j++) {
			pos.at<float>(0, j) = path.at<float>(i, j) + step.at<float>(j, 0);

		}
		//std::cout << pos << std::endl;
		path.push_back(pos);
	}

	return path;

}
Mat Create(Mat pastRoute, Mat Routenow, int routeindex) {
	Mat tmp;


	tmp.push_back(pastRoute);

	for (int i = routeindex; i < Routenow.rows; i++) {
		tmp.push_back(Routenow.row(i));
	}

	return tmp;
}
void framebuffer_size_callback(GLFWwindow*, int, int);
void processInput(GLFWwindow*);
void mouse_callback(GLFWwindow*, double, double);
void scroll_callback(GLFWwindow*, double, double);

const unsigned int SCR_WIDTH = 800;
const unsigned int SCR_HEIGHT = 600;
// camera
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
float deltaTime;
Cylinder cylinder[6];
GLuint VAO[6];
GLuint VBO[6];
GLuint EBO[6];
GLuint VBOline;
GLuint VAOline;

//cv::Mat pathcreate(cv::Mat start, cv::Mat end, int sampleRange) {
//	cv::Mat P1 = start;
//	cv::Mat P2 = end;
//	cv::Mat step;
//	for (int i = 0; i < P1.cols; i++) {
//		step.push_back((P1.col(i) - P2.col(i)) / sampleRange);
//		//std::cout << step << std::endl;
//	}
//	cv::Mat path;
//	cv::Mat pos = (cv::Mat_<float>(1, 6) << 0, 0, 0, 0, 0, 0);
//	path.push_back(P2);
//	for (int i = 0; i < sampleRange; i++) {
//
//		for (int j = 0; j < 6; j++) {
//			pos.at<float>(0, j) = path.at<float>(i, j) + step.at<float>(j, 0);
//
//		}
//		//std::cout << pos << std::endl;
//		path.push_back(pos);
//	}
//
//	return path;
//
//}
int main()
{
	////First Row is Min, Second Row is Max Range
	////chrono::high_resolution_clock::time_point J1 = chrono::high_resolution_clock::now();
	glfwInit();
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);


	//glfw window creation
	GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "LearnOpenGL", NULL, NULL);
	if (window == NULL)
	{
		std::cout << "Failed to create GLFW window" << std::endl;
		glfwTerminate();
		return -1;
	}
	glfwMakeContextCurrent(window);
	glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
	glfwSetCursorPosCallback(window, mouse_callback);
	glfwSetScrollCallback(window, scroll_callback);

	// tell GLFW to capture our mouse
	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

	//glad: load all OpenGL function pointers
	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
	{
		std::cout << "Failed to initialize GLAD" << std::endl;
		return -1;
	}

	Draw myRobot;
	myRobot.init();
	////////////////////////////////////
	char filename[256];
	FILE* outfile;
	Mat JointRangeMatrix = (Mat_<float>(2, 6) << -20.81, -120, 75, -175, -125, -350, 			//各軸角度限制
		185, -72.73, 115, 175, 125, 350);


	Ptr<ml::TrainData> data = ml::TrainData::loadFromCSV("path/1116.csv", 0, -2, 0); //讀取路徑(可能會改
	Mat RouteSample = data->getTrainSamples();


	Mat DataOffset = Mat::zeros(1, RouteSample.cols, CV_32F);

	copyMakeBorder(DataOffset, DataOffset, 0, RouteSample.rows - 1, 0, 0, BORDER_REFLECT); //角度偏移用(目前不會用到

	RouteSample = RouteSample + DataOffset;
	//Mat start = (Mat_<float>(1, 6) << 100.098, -30.849, -16.200, -13.750, -38.611, 18.207);
	//Mat end = (Mat_<float>(1, 6) << -100.098, -30.849, -16.200, -13.750, -38.611, 18.207);

	// Mat start = (Mat_<float>(1, 6) << -14.56, -94.25, 112.6, 0, 75, 0.0);
	// Mat end = (Mat_<float>(1, 6) << 180.25, -94.25, 112.6, 0, 75, 0.0);//軸1:27.25
	Mat start = (Mat_<float>(1, 6) << 40, -94.25, 80, 0, 75, 0);   // -675 75 375 ;-675 0 575
	Mat end = (Mat_<float>(1, 6) << 160, -94.25, 80, 0, 75, 0);
	Mat RouteSample_ = pathcreate(start, end, 250);
	cout << RouteSample_ << endl;
	sprintf_s(filename, "./result/tmp.csv");
	fopen_s(&outfile, filename, "w");
	//NewtonOptimize PosOptimizer(JointRangeMatrix, RouteSample); //宣告物件
	//vector<Mat> Path = PosOptimizer.RoutePlanning(RouteSample, 7 - 1, 10 - 1, 2 - 1); //執行路徑計算
	//Mat tmp=Mat_<float>(0,6);
	bool flag = true; int posCount = 1;
	NewtonOptimize PosOptimizer(JointRangeMatrix, RouteSample_);
	while (true) {
		string a, b, c;
		cout << "cube: ";
		cin >> a >> b >> c;
		std::string pathing = "result/box " + a;
		pathing = pathing + "-";
		pathing = pathing + b;
		pathing = pathing + "-";
		pathing = pathing + c;
		cout << pathing << endl;

		//int x_ = stoi(a);
		//int y_ = stoi(b);
		//int z_ = stoi(c);
		float x_ = stof(a);
		float y_ = stof(b);
		float z_ = stof(c);
		
		chrono::high_resolution_clock::time_point J1 = chrono::high_resolution_clock::now();
		vector<Mat> Path = PosOptimizer.RoutePlanning(x_, y_ , z_ ); //執行路徑計算
		//vector<Mat> Path = PosOptimizer.RoutePlanning(0, 0, 0, point); //執行路徑計算
		//PosOptimizer.setCube(x_, y_, z_);

		//vector<Mat> Path = PosOptimizer.RoutePlanning(x_ - 1, y_ - 1, z_ - 1); //執行路徑計算
		chrono::high_resolution_clock::time_point J2 = chrono::high_resolution_clock::now();
		std::cout << chrono::duration_cast<chrono::milliseconds>(J2 - J1).count() << " ms" << endl;
		cv::Mat result = PosOptimizer.showPath();
		int row = result.rows;
		int col = result.cols;
		vector<vector<float>> angle;
		for (int i = 0; i < result.rows; i++) {
			std::vector<float> tmp;
			for (int j = 0; j < result.cols; j++) {
				tmp.push_back(result.at<float>(i, j));
			}
			angle.push_back(tmp);
		}
		int pos = 0;
		GLfloat newVertices[3];
		while (!glfwWindowShouldClose(window)&&pos < row) {
			auto vertices = PosOptimizer.forwardKinematic(result.row(pos));
			//cout << vertices[0] << vertices[1] << vertices[2] << endl;
			newVertices[0] = vertices[0]/1000-0.1f;
			newVertices[1] = vertices[1]/1000;
			newVertices[2] = vertices[2]/1000;
			angle [pos] = {0 ,0,0,0,0,0};
			myRobot.showObject(pos, angle[pos], newVertices, window);
			pos++;
			//if (pos >= row) {
			//	pos = 0;
			//}

		}
	}
	system("pause");
	myRobot.terminate();





	float lenght = 0;
	//Ptr<ml::TrainData> re = ml::TrainData::loadFromCSV("result/dynamic.csv", 0, -2, 0); //讀取路徑(可能會改
	//Mat RouteResult = re->getTrainSamples();
	//for (int i = 0; i < RouteResult.rows; i++) {
	//	cout << RouteResult.row(i) << endl;
	//	if (i > 0) {
	//		lenght += cv::norm(RouteResult.row(i), RouteResult.row(i - 1));
	//	}
	//}
	//cout << lenght << endl;
	
	return 0;
}
void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
	glViewport(0, 0, width, height);
}

// process all input: query GLFW whether relevant keys are pressed/released this frame and react accordingly
void processInput(GLFWwindow* window)
{
	const float cameraSpeed = static_cast<float>(2.5 * deltaTime); // adjust accordingly
	if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
		cameraPos -= cameraSpeed * cameraUp;
	if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
		cameraPos += cameraSpeed * cameraUp;
	if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
		cameraPos -= glm::normalize(glm::cross(cameraFront, cameraUp)) * cameraSpeed;
	if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
		cameraPos += glm::normalize(glm::cross(cameraFront, cameraUp)) * cameraSpeed;
	if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
		glfwSetWindowShouldClose(window, true);

}

void mouse_callback(GLFWwindow* window, double xposIn, double yposIn)
{
	float xpos = static_cast<float>(xposIn);
	float ypos = static_cast<float>(yposIn);
	
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

	float sensitivity = 0.1f;
	xoffset *= sensitivity;
	yoffset *= sensitivity;

	yaw += xoffset;
	pitch += yoffset;

	if (pitch > 89.0f)
		pitch = 89.0f;
	if (pitch < -89.0f)
		pitch = -89.0f;

	glm::vec3 direction;
	direction.x = cos(glm::radians(yaw)) * cos(glm::radians(pitch));
	direction.y = sin(glm::radians(pitch));
	direction.z = sin(glm::radians(yaw)) * cos(glm::radians(pitch));
	cameraFront = glm::normalize(direction);
}

void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
	fov -= (float)yoffset;
	if (fov < 1.0f)
		fov = 1.0f;
	if (fov > 45.0f)
		fov = 45.0f;
}

