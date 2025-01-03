#pragma once
#include	<stdio.h>
#include	<stdlib.h>
#include <iostream>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <opencv2/opencv.hpp>
#include<vector>
#define	RESOLUTION		150

/* LWH=[-900 900 1200]; */
#define	LWH_XY_MIN		-900    
#define	LWH_XY_MAX		900
#define	LWH_Z_MIN		0
#define	LWH_Z_MAX		1200
#define     _USE_MATH_DEFINES
#include	<math.h>
#define	CountOfArray(_array)		(sizeof(_array) / sizeof(_array[0]))

#define	LWH_SIZE_XY		(((LWH_XY_MAX - LWH_XY_MIN) / RESOLUTION) + 1)
#define	LWH_SIZE_Z		(((LWH_Z_MAX - LWH_Z_MIN) / RESOLUTION) + 1)


class NewtonOptimize
{
public: //公開宣告(主程式可存取
	NewtonOptimize(cv::Mat JointRangeMatrix, cv::Mat RouteSample);


	~NewtonOptimize();
	typedef struct	T3DPoint {
		long	x, y, z;
	} T3DPoint;

	typedef struct	T3DValue {
		float	x, y, z;
	} T3DValue;

	typedef float	TArray4[4];

	typedef TArray4	TArray2X4[2];

	typedef TArray4	TArray4X4[4];

	T3DPoint		WS[LWH_SIZE_Z][LWH_SIZE_XY][LWH_SIZE_XY];//工作空間Workspace

	T3DPoint		box[LWH_SIZE_Z - 1][LWH_SIZE_XY - 1][LWH_SIZE_XY - 1][8];//格子

	T3DValue		dectectPt[LWH_SIZE_Z - 1][LWH_SIZE_XY - 1][LWH_SIZE_XY - 1];//探測點(格子中心

	//危險值緩存
	static float    sjResult[CountOfArray(box)][CountOfArray(box[0])][CountOfArray(box[0][0])];
	void SampleRange(float PercentageOfPath);
	int returnIndex();
	//std::vector<cv::Mat> RouteCheck(cv::Mat CurrentPath, int cubeX, int cubeY, int cubeZ, float SafeThreshold = 0.4f);
	//std::vector<cv::Mat> RoutePlanning(cv::Mat SafeScoreOutOfLimit);//路徑計算
	std::vector<cv::Mat> RoutePlanning(int cubeX, int cubeY, int cubeZ, T3DValue point = { 0,0,0 }, float SafeThreshold = 0.4f);
	std::vector<cv::Mat> RoutePlanning(float cubeX, float cubeY, float cubeZ, T3DValue point = { 0,0,0 }, float SafeThreshold = 0.4f);
	std::vector<cv::Mat> RoutePlanning(cv::Mat CurrentPath, int InputCubeX, int InputCubeY, int InputCubeZ, float SafeThreshold=0.4f);
	cv::Mat showPath();
	std::vector<float > forwardKinematic(cv::Mat PathData);


private: //不公開宣告(主程式不可存取

	enum TargetVector //類別(供矩陣切割用
	{
		Xm = 0,//6
		Xm2,//6
		Xm3,//6
		lota,//3
		lota2,//3
		lota3,//3
		S,//3
		S2,//3
		S3,//3
		mu,//6
		mu2,//6
		mu3,//6
		U,//6
		U2,//6
		U3,//6
		tau,//6
		tau2,//6
		tau3,//6
		V,//6
		V2,//6
		V3,//6
	};


	enum Direction //類別(供微分使用
	{
		Plus,
		Minus
	};

	enum NormalizeStatus {
		Forward,
		Backward
	};
	cv::Mat CurrentPath_;
	float TargetSafeThreshold; //(安全閥值)
	cv::Mat JointRangeMatrix; //角度限制
	int _index;
	std::size_t JointCount = 7; //關節數量
	std::size_t MaxIterLimit = 100; //迭代極限
	//float MinDistLimit = 0.4; //最佳化收斂條件(過後使用
	float Delta = 1; //0.0002 one Degree  //微分分母
	//cv::Mat Deltax = (cv::Mat_<float>(1, 6) << 0.001, 0.001, 0.001,0.001, 0.001,0.001);
	cv::Mat Deltax = (cv::Mat_<float>(1, 6) <<1,1,1,1,1,1);
	cv::Mat deltaX;
	int MinIndex1 = 0;
	int MaxIndex1 = 0;
	//權重
	float f_weight = 1.00;
	float t_weight = 0.50;
	float h_weight = 0.50;
	float w1 = 0.50;
	float w2 = 0.50;
	float m1 = 1.00;
	float m2 = 1.00;
	float m3 = 1.00;
	float new_weight = 0;

	//最佳化變數
	//float slack = 1;
	//float lagragian = 0.5;
	float lagragian_l = 10.0;
	float slack_S = 0.1;
	float lagragian_u = 0.1;
	float slack_U = 0.1;
	float lagragian_t = 0.1;
	float slack_V = 0.1;
	int Dcount=0;
	//存檔用
	char filename[256];
	FILE* outFile;

	float PathSampleRange = 0.25;//路徑取樣
	std::vector<float> CubeSafeScore; //安全值

	//adjustment
	int gk_multiplier = 1;
	////危險格子
	//int cubeX;
	//int cubeY;
	//int cubeZ;
	float cubeX;
	float cubeY;
	float cubeZ;
	int CubeCount = 1;
	cv::Mat CurrentPath;
	//定義宣告(方便計算
	
	//矩陣計算

	static void			ArrayMul(TArray4X4 a, TArray4X4 b, TArray4X4& result);
	static void			ArrayMul(TArray4X4 a, TArray2X4 b, TArray2X4& result);
	static void			ArrayMul(TArray4X4 a, const float* bPtr, float* resultPtr);
	static T3DValue		Sub(T3DValue a, T3DValue b);
	static float		Dot(T3DValue a, T3DValue b);
	static T3DValue		Cross(T3DValue a, T3DValue b);
	static float		Norm(T3DValue a);
	cv::Mat ForwardKinematics(cv::Mat joint);
	//計算最佳化變數
	cv::Mat Get_gk(cv::Mat CurrentPos, cv::Mat S);
	cv::Mat Get_ui(cv::Mat CurrentPos, cv::Mat U);
	cv::Mat Get_vi(cv::Mat CurrentPos, cv::Mat V);
	cv::Mat pathcreate(cv::Mat start, cv::Mat end,int sampleRange);
	cv::Mat Create(cv::Mat pastRoute, cv::Mat Routenow, int routeindex);
	cv::Mat ExtractSubVector(cv::Mat Data, TargetVector Target);//提取矩陣切割
	cv::Mat Normalize(cv::Mat Src, NormalizeStatus flag);//標準化
	cv::Mat DeltaNormalize(cv::Mat Src, NormalizeStatus flag);
	float CalcD(std::size_t i, cv::Mat Pos);//標準化後計算危險值
	std::vector<float> CalcDJ(cv::Mat Pos);
	//cv::Mat OptimizeSinglePos(cv::Mat CurrentPos, cv::Mat StartPos, cv::Mat EndPos);//計算單筆最佳化
	cv::Mat OptimizeSinglePos(cv::Mat CurrentPos, cv::Mat StartPos, cv::Mat EndPos);//計算單筆最佳化
	cv::Mat OptimizePos(std::vector<int> points);
	float FunctionXm(cv::Mat CurrentPos, cv::Mat OriPos, cv::Mat StartPos, cv::Mat EndPos);//Objective Function
	float FunctionXm(cv::Mat CurrentPos1, cv::Mat OriPos1, cv::Mat CurrentPos2, cv::Mat OriPos2, cv::Mat CurrentPos3, cv::Mat OriPos3, cv::Mat StartPos, cv::Mat EndPos);
	void setIndex(int index);
	


	//微分
	cv::Mat PosDelta(cv::Mat CurrentPos, int i, int j);
	cv::Mat PosDeltaM(cv::Mat CurrentPos, int i, int j);
	cv::Mat PosDeltaPM(cv::Mat CurrentPos, int i, int j);
	cv::Mat PosDeltaMP(cv::Mat CurrentPos, int i, int j);
	cv::Mat PosDelta(cv::Mat CurrentPos, int i, Direction Dir);
	cv::Mat PosDeltaX(cv::Mat CurrentPos, int i, Direction Dir);
	cv::Mat PosDeltaX(cv::Mat CurrentPos, int i, int j);
	//最佳化
	//cv::Mat CalcHessian(cv::Mat CurrentPos, cv::Mat OriPos, cv::Mat StartPos, cv::Mat EndPos);//海森矩陣
	cv::Mat CalcHessian(cv::Mat CurrentPos, cv::Mat OriPos1, cv::Mat OriPos2, cv::Mat OriPos3, cv::Mat StartPos, cv::Mat EndPos);
	cv::Mat CalcGradient(cv::Mat CurrentPos, cv::Mat PreviousPos, cv::Mat StartPos, cv::Mat EndPos);//梯度
	//cv::Mat CalcGradient(cv::Mat CurrentPos, cv::Mat OriPos1, cv::Mat OriPos2, cv::Mat OriPos3, cv::Mat StartPos, cv::Mat EndPos);
	//cv::Mat CalcJointWeight(cv::Mat CurrentPos, cv::Mat StartPos, cv::Mat EndPos);//計算權重(目前沒用

	//創造工作空間(計算危險值用
	void FKinematic(cv::Mat PathData);
	void	 MakeWorkSpace(void);
	std::vector<float>	RobotArm(cv::Mat PathData, int cubeX, int cubeY, int cubeZ,T3DValue point = {0,0,0});//計算危險值
	std::vector<float>	RobotArm(cv::Mat PathData, float cubeX, float cubeY, float cubeZ, T3DValue point = { 0,0,0 });//計算危險值
	//std::vector<std::vector<std::vector<std::vector<float>>>>	RobotArm(cv::Mat PathData, int cubeX, int cubeY, int cubeZ,T3DValue point={0,0,0});//計算危險值
	std::vector<float>  RobotArmJ(cv::Mat PathData, int XXX, int YYY, int ZZZ, T3DValue point = { 0,0,0 });
	std::vector<float>  RobotArmJ(cv::Mat PathData, float XXX, float YYY, float ZZZ, T3DValue point = { 0,0,0 });

	//運動學
	void multiple(std::vector<std::vector<float>>& rottheta, std::vector<std::vector<float>>& alpha, std::vector<std::vector<float>>& rot01);
	cv::Mat InvKinematic(std::vector<float> pos);
};
