#include "NewtonUpdate.h"

#include <Eigen/Dense>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <utility>
#include <sstream>
#include <opencv2/core/eigen.hpp>
#include <stdio.h>


#include <opencv2/opencv.hpp>

//#include "Function.h"
using namespace std;
using namespace cv;
using namespace ml;
using namespace Eigen;

//初始化
NewtonOptimize::NewtonOptimize(Mat NormalizeMatrix, Mat CurrentPath_)
{
	cout << "Start Loading Model..." << endl;
	MakeWorkSpace();
	cout << "Model is Loaded!!" << endl;
	JointRangeMatrix = NormalizeMatrix.clone();
	deltaX = Deltax;
	CurrentPath = CurrentPath_;
}

NewtonOptimize::~NewtonOptimize()
{
}

//取樣範圍與索引設置
void NewtonOptimize::SampleRange(float PercentageOfPath) {
	if (PercentageOfPath >= 1) {
		cout << "PercentageOfPath Can't Bigger Than 1" << endl;
		return;
	}
	PathSampleRange = PercentageOfPath;
}
void NewtonOptimize::setIndex(int index_) { _index = index_; }
int NewtonOptimize::returnIndex() { return _index; } //設定取樣路徑的百分比，確保所取樣的路徑部分不會超過整條路徑的範圍

//路徑規劃
vector<Mat> NewtonOptimize::RoutePlanning(float InputCubeX, float InputCubeY, float InputCubeZ, T3DValue point, float SafeThreshold) {

	std::clock_t begin = std::clock();
	chrono::high_resolution_clock::time_point J3 = chrono::high_resolution_clock::now();

	std::vector<float> AllSafeScore;
	cubeX = InputCubeX; //點座標
	cubeY = InputCubeY;
	cubeZ = InputCubeZ;
	AllSafeScore = RobotArm(CurrentPath, InputCubeX, InputCubeY, InputCubeZ);
	//計算每個路徑點的安全分數。根據機械手臂當前的位置與輸入的障礙物座標進行計算，返回每個點的安全得分。
	JointCount = CurrentPath.cols;
	TargetSafeThreshold = SafeThreshold; //安全值


	//auto AllSafeScore = RobotArm(CurrentPath);

	CubeSafeScore.clear();

	for (size_t i = 0; i < AllSafeScore.size(); i++)
	{
		CubeSafeScore.push_back(AllSafeScore[i]);
	}

	Mat TotalSafeScore(CubeSafeScore);
	Mat SafeScoreOutOfLimit, SafeScoreOutOfLimitIndex, SafeScoreCollide, SafeScoreCollideIndex;

	threshold(TotalSafeScore, SafeScoreOutOfLimit, SafeThreshold, 1, THRESH_BINARY);
	threshold(TotalSafeScore, SafeScoreCollide, 0.5f, 1, THRESH_BINARY);
	//用於將安全得分與安全閾值進行比較，並標記出超出閾值的部分，這些部分代表了可能會發生碰撞的點。
	findNonZero(SafeScoreOutOfLimit, SafeScoreOutOfLimitIndex);
	//找到所有超過安全閾值的點，並將這些點的索引儲存在 SafeScoreOutOfLimitIndex 中。
	findNonZero(SafeScoreCollide, SafeScoreCollideIndex);
	extractChannel(SafeScoreOutOfLimitIndex, SafeScoreOutOfLimitIndex, 1);

	double idx, min;
	minMaxIdx(TotalSafeScore, &min, &idx, &MinIndex1, &MaxIndex1); //找到安全得分的最小值和最大值對應的索引。這些索引用來選擇新的目標點。

	double MaxIndex, MinIndex;
	minMaxIdx(SafeScoreOutOfLimitIndex, &MinIndex, &MaxIndex);

	double mean = 10;
	int quartile1, quartile3;

	for (int i = MinIndex; i < MaxIndex1; i++) {
		double q1 = abs((TotalSafeScore.at<float>(i, 0)) - ((0.4 + idx) / 2));
		if (q1 < mean) {
			quartile1 = i;
			mean = q1;
		}

	}
	mean = 10;
	for (int i = MaxIndex; i > MaxIndex1; i--) {
		double q3 = abs((TotalSafeScore.at<float>(i, 0)) - ((0.4 + idx) / 2));
		if (q3 < mean) {
			quartile3 = i;
			mean = q3;
		}


	}
	double MiddleIndex = (MinIndex + MaxIndex) * 0.5;
	vector<int> TargetPos = { int(MinIndex) , //根據計算出的四分位數 quartile1 和 quartile3，選擇新的目標點來進行路徑優化，並將這些索引加入 TargetPos。
		//int(MinIndex + (MiddleIndex - MinIndex) * 0.25),
							//int((MinIndex + MiddleIndex) * 0.5),
							quartile1,
		//int(MinIndex + (MiddleIndex - MinIndex) * 0.75),
							//int(MiddleIndex),
							MaxIndex1,
		//int(MiddleIndex + (MaxIndex - MiddleIndex) * 0.25),
							//int((MiddleIndex + MaxIndex) * 0.5),
							quartile3,
		//int(MiddleIndex + (MaxIndex - MiddleIndex) * 0.75),
							int(MaxIndex) };
	setIndex(MaxIndex);
	cout << "Target Pos: ";
	for (const auto& item : TargetPos) {
		cout << item << ", ";
	}
	cout << endl;

	vector<Mat> Result;
	Mat StartPos = CurrentPath.row(TargetPos.front());
	Result.push_back(StartPos);
	cout << "Start Pos:" << StartPos << endl;

	Mat EndPos = CurrentPath.row(TargetPos.back());

	auto SafePos = OptimizePos(TargetPos);
	//呼叫 OptimizePos 函數，對選定的路徑點進行優化。
	//OptimizePos 函數使用的是牛頓法或其他優化方法，通過計算每個點的位置更新來避免障礙物。
	for (int i = 0; i < SafePos.rows; i++) {
		Result.push_back(SafePos.row(i));
		cout << "安全點" << SafePos.row(i) << endl;
	}


	Result.push_back(EndPos);
	cout << "End Pos:" << EndPos << endl;
	Mat new_path;

	new_path.push_back(CurrentPath.rowRange(0, (int)MinIndex));

	//new_path.push_back(pathcreate(Result[0], CurrentPath.row(0), 100));
	for (int k = 0; k < Result.size() - 1; k++) {
		new_path.push_back(pathcreate(Result[k + 1], Result[k], 20));
	}

	new_path.push_back(CurrentPath.rowRange(int(MaxIndex), CurrentPath.rows - 1));


	//new_path.push_back(pathcreate(CurrentPath.row(CurrentPath.rows-1), Result[4], 100));
	CurrentPath = new_path; //將新的路徑賦值給 CurrentPath，這樣機械手臂就能沿著新的安全路徑移動。
	cout << new_path << endl;
	return Result;
}
Mat NewtonOptimize::showPath() {
	return CurrentPath;
}

//根據當前機械手臂的關節位置 CurrentPos 和松弛變數 S，計算一個矩陣 gk。
//這個 gk 可能代表某種約束條件的梯度值或是優化過程中的目標函數。
Mat NewtonOptimize::Get_gk(Mat CurrentPos, Mat S) {
	Mat gk;
	vector<float> df = CalcDJ(CurrentPos);
	for (size_t k = 0; k < JointCount; k++)
	{
		gk.push_back((float)(df[k] - 0.4 + pow(S.at<float>(0, k), 2)));
	}
	return gk;
}

//根據當前的機械手臂關節位置CurrentPos，先將其每個元素減去 1，然後將 U 矩陣的平方加到結果中。返回一個新的矩陣ui
Mat NewtonOptimize::Get_ui(Mat CurrentPos, Mat U) {
	Mat U_pow;
	pow(U, 2, U_pow);

	Mat ui = CurrentPos - Mat::ones(CurrentPos.size(), CurrentPos.type()) + U_pow;

	return ui;
}


Mat NewtonOptimize::Get_vi(Mat CurrentPos, Mat V) {
	Mat V_pow;
	pow(V, 2, V_pow);

	Mat vi = Mat::zeros(CurrentPos.size(), CurrentPos.type()) - CurrentPos + V_pow;

	return vi;
}

//這個函數根據輸入的 TargetVector 值，從矩陣 Data 中提取對應的子向量。
//這個子向量代表機械手臂的不同參數，例如關節位置、速度、松弛變數等。
Mat NewtonOptimize::ExtractSubVector(Mat Data, TargetVector Target) {
	int StartCol, EndCol;

	switch (Target)
	{
	case NewtonOptimize::Xm:
		StartCol = 0;
		EndCol = JointCount;
		break;
	case NewtonOptimize::Xm2:
		StartCol = JointCount;
		EndCol = StartCol + JointCount;
		break;
	case NewtonOptimize::Xm3:
		StartCol = 2 * JointCount;
		EndCol = StartCol + JointCount;
		break;
	case NewtonOptimize::lota:
		StartCol = 3 * JointCount;
		EndCol = StartCol + JointCount;
		break;
	case NewtonOptimize::lota2:
		StartCol = 4 * JointCount;
		EndCol = StartCol + JointCount;
		break;
	case NewtonOptimize::lota3:
		StartCol = 5 * JointCount;
		EndCol = StartCol + JointCount;
		break;
	case NewtonOptimize::S:
		StartCol = 6 * JointCount;
		EndCol = StartCol + JointCount;
		break;
	case NewtonOptimize::S2:
		StartCol = 7 * JointCount;
		EndCol = StartCol + JointCount;
		break;
	case NewtonOptimize::S3:
		StartCol = 8 * JointCount;
		EndCol = StartCol + JointCount;
		break;

	default:
		StartCol = 0;
		EndCol = 0;
		break;
	}
	return Data.colRange(StartCol, EndCol);
}

//對輸入的矩陣 Src 進行正規化或反正規化操作
Mat NewtonOptimize::Normalize(Mat Src, NormalizeStatus flag) {
	Mat Res;
	Mat Range = JointRangeMatrix.row(1) - JointRangeMatrix.row(0);
	if (flag == Forward) {
		divide((Src - JointRangeMatrix.row(0)), Range, Res);
		return Res;
	}
	else {
		multiply(Src, Range, Res);
		return Res + JointRangeMatrix.row(0);
	}
}

//用來處理變化量或增量（Delta）的正規化和反正規化
Mat NewtonOptimize::DeltaNormalize(Mat Src, NormalizeStatus flag) {

	Mat Res;
	Mat Range = JointRangeMatrix.row(1) - JointRangeMatrix.row(0);
	if (flag == Forward)
	{
		divide(Src, Range, Res);
		return Res;
	}
	else {
		multiply(Src, Range, Res);
		return Res;
	}
}

//用來計算機械手臂當前位置與障礙物之間的距離或安全分數，並返回一個單一值
float NewtonOptimize::CalcD(size_t i, Mat Pos) {
	//Pos = Normalize(Pos, Backward);
	Dcount++;
	//cout << count << endl;
	auto Value = RobotArm(Pos, cubeX, cubeY, cubeZ);

	return Value[0];
}

//用來計算機械手臂各個關節在當前位置的相關值，並返回一個包含多個關節結果的向量
std::vector<float>  NewtonOptimize::CalcDJ(Mat Pos) {
	//Pos = Normalize(Pos, Backward);
	auto Value = RobotArmJ(Pos, cubeX, cubeY, cubeZ);
	return Value;
}
//牛頓法計算
Mat NewtonOptimize::OptimizePos(vector<int> TargetPos) {
	//if (fopen_s(&outFile, filename, "w"), outFile != NULL) {
	cout << "check" << endl;
	Mat CurrentPos;

	Mat StartPos = CurrentPath.row(TargetPos[0]);
	Mat EndPos = CurrentPath.row(TargetPos[4]);
	Mat Xm;
	for (int i = 1; i < TargetPos.size() - 1; i++) {
		CurrentPos.push_back(CurrentPath.row(TargetPos[i]));
		for (int j = 0; j < JointCount; j++) {
			Xm.push_back(CurrentPath.at<float>(TargetPos[i], j));
		}

	}
	cout << CurrentPos << endl;
	Mat l1 = Mat::ones(1, JointCount, CV_32F);
	Mat S1 = Mat::ones(1, JointCount, CV_32F);
	Mat l2 = Mat::ones(1, JointCount, CV_32F);
	Mat S2 = Mat::ones(1, JointCount, CV_32F);
	Mat l3 = Mat::ones(1, JointCount, CV_32F);
	Mat S3 = Mat::ones(1, JointCount, CV_32F);


	l1 = l1 * lagragian_l;
	S1 = S1 * slack_S;
	l2 = l2 * lagragian_l;
	S2 = S2 * slack_S;
	l3 = l3 * lagragian_l;
	S3 = S3 * slack_S;

	Xm.push_back(l1.t());
	Xm.push_back(l2.t());
	Xm.push_back(l3.t());
	Xm.push_back(S1.t());
	Xm.push_back(S2.t());
	Xm.push_back(S3.t());

	Mat PreviousIterPos = Xm.clone();
	//cout << dectectPt[cubeZ][cubeY][cubeX].x << ", " << dectectPt[cubeZ][cubeY][cubeX].y << ", " << dectectPt[cubeZ][cubeY][cubeX].z << endl;
	float ResultSafetyScore = 0;
	for (size_t i = 1; i < MaxIterLimit; i++)
	{

		float ResultSafetyScoretmp = 0;
		float ResultSafetyScoretmp1 = 0;
		float ResultSafetyScoretmp2 = 0;
		float ResultSafetyScoretmp3 = 0;

		//梯度計算
		Mat GradientMatrix = CalcGradient(Xm, CurrentPos, StartPos, EndPos);
		//海森矩陣計算
		Mat HessianMatrix = CalcHessian(Xm, CurrentPos.row(0), CurrentPos.row(1), CurrentPos.row(2), StartPos, EndPos);

		Mat NewIterPos = Xm - HessianMatrix.inv() * GradientMatrix; //牛頓法公式

		// 透過 CalcGradient 函數計算梯度，然後透過 CalcHessian 計算海森矩陣。最後根據這兩者的結果，更新變量位置。
		//牛頓法利用海森矩陣的逆矩陣來加速收斂
		auto NewPosXm1 = ExtractSubVector(NewIterPos.t(), TargetVector::Xm);
		auto NewPosXm2 = ExtractSubVector(NewIterPos.t(), TargetVector::Xm2);
		auto NewPosXm3 = ExtractSubVector(NewIterPos.t(), TargetVector::Xm3);
		Mat NewPosXm;
		NewPosXm.push_back(NewPosXm1);
		NewPosXm.push_back(NewPosXm2);
		NewPosXm.push_back(NewPosXm3);
		vector<float> safeScore = CalcDJ(NewPosXm1);
		for (int k = 0; k < JointCount; k++) {
			cout << safeScore[k] << ", ";
		}cout << endl;

		ResultSafetyScoretmp1 = CalcD(i, NewPosXm1);
		ResultSafetyScoretmp2 = CalcD(i, NewPosXm2);
		ResultSafetyScoretmp3 = CalcD(i, NewPosXm3);
		for (int j = 0; j < 6; j++) {
			//cout << safeScore[j] << ", ";
		}//cout << NewPosXm1<<endl;

		//cout << ResultSafetyScoretmp1 << " " << ResultSafetyScoretmp2 << " " << ResultSafetyScoretmp3 << endl;
		float tmp = 0;
		for (int j = 0; j < 3; j++) {
			ResultSafetyScoretmp = CalcD(i, NewPosXm.row(j));
			//cout << ResultSafetyScoretmp << endl;
			if (tmp < ResultSafetyScoretmp) {
				tmp = ResultSafetyScoretmp;
			}
		}

		if (tmp < 0.41f && abs(tmp - ResultSafetyScore) < 0.01) {
			cout << tmp << endl;
			ResultSafetyScore = 0;
			return NewPosXm;

		}

		ResultSafetyScore = tmp;
		PreviousIterPos = Xm.clone();
		Xm = NewIterPos.clone();
	}
	//}
	cout << "shit" << endl;
	return Mat();
}

//計算當前路徑點與目標位置、起點、終點，以及中繼點之間的距離平方差，並對這些平方差進行加權求和。
float NewtonOptimize::FunctionXm(Mat CurrentPos1, Mat OriPos1, Mat CurrentPos2, Mat OriPos2, Mat CurrentPos3, Mat OriPos3, Mat StartPos, Mat EndPos) {
	Mat PowBetween1, PowBetween2, PowCurrentStart, PowCurrentEnd, PowPos1, PowPos2, PowPos3;
	pow(norm(CurrentPos1, OriPos1), 2, PowPos1);
	pow(norm(CurrentPos2, OriPos2), 2, PowPos2);
	pow(norm(CurrentPos3, OriPos3), 2, PowPos3);
	pow(norm(CurrentPos1, StartPos), 2, PowCurrentStart);
	pow(norm(CurrentPos3, EndPos), 2, PowCurrentEnd);
	pow(norm(CurrentPos1, CurrentPos2), 2, PowBetween1);
	pow(norm(CurrentPos2, CurrentPos3), 2, PowBetween2);

	return sum(m1 * PowPos1 + m2 * PowPos2 + m3 * PowPos3 + h_weight * PowCurrentStart + t_weight * PowCurrentEnd + w1 * PowBetween1 + w2 * PowBetween2)[0];
}


//反覆使用來評估路徑規劃過程中的每個中繼點，並根據結果調整路徑
float NewtonOptimize::FunctionXm(Mat CurrentPos, Mat OriPos, Mat StartPos, Mat EndPos) {
	Mat PowCurrentOri, PowCurrentStart, PowCurrentEnd, OLDMPFC, NEWMPFC, CenterPoint;
	pow(norm(CurrentPos, OriPos), 2, PowCurrentOri);
	pow(norm(CurrentPos, StartPos), 2, PowCurrentStart);
	pow(norm(CurrentPos, EndPos), 2, PowCurrentEnd);

	return  sum(f_weight * PowCurrentOri + h_weight * PowCurrentStart + t_weight * PowCurrentEnd)[0];
}

//對 CurrentPos 進行微小的變動，生成一個新的位置矩陣 Out，並將這些變動反映在機械手臂的第 i 和第 j 個關節上。
Mat NewtonOptimize::PosDelta(Mat CurrentPos, int i, int j) {
	Mat Out = CurrentPos.clone();
	Out.at<float>(0, i) += deltaX.at<float>(0, i);
	Out.at<float>(0, j) += deltaX.at<float>(0, j);
	return Out;
}

//根據指定的方向（Direction），對 CurrentPos 矩陣中第 i 個關節的坐標進行增量調整，並返回調整後的新位置矩陣。
Mat NewtonOptimize::PosDelta(Mat CurrentPos, int i, Direction Dir) {
	Mat Out = CurrentPos.clone();
	if (Dir == Plus) {
		Out.at<float>(0, i) += deltaX.at<float>(0, i);
	}
	else {
		Out.at<float>(0, i) -= deltaX.at<float>(0, i);
	}
	return Out;
}
//Mat NewtonOptimize::CalcGradient(Mat CurrentPos, Mat OriPos1, Mat OriPos2, Mat OriPos3, Mat StartPos, Mat EndPos) {
//	CurrentPos = CurrentPos.t();
//
//
//	Mat Gradient;
//	Mat XmVector1 = ExtractSubVector(CurrentPos, Xm);
//	Mat SVector1 = ExtractSubVector(CurrentPos, S);
//	Mat UVector1 = ExtractSubVector(CurrentPos, U);
//	Mat VVector1 = ExtractSubVector(CurrentPos, V);
//	Mat lotaVector1 = ExtractSubVector(CurrentPos, lota);
//	Mat muVector1 = ExtractSubVector(CurrentPos, mu);
//	Mat tauVector1 = ExtractSubVector(CurrentPos, tau);
//
//}

//計算機械手臂路徑規劃的梯度矩陣，用於牛頓法的優化過程。
//機械手臂在不同位置的變化（通過增量 deltaX）和拉格朗日乘數結合，計算出梯度值，最終為優化算法提供方向。
Mat NewtonOptimize::CalcGradient(Mat CurrentPos, Mat OriPos, Mat StartPos, Mat EndPos) {
	CurrentPos = CurrentPos.t();

	Mat Gradient;
	Mat XmVector1 = ExtractSubVector(CurrentPos, Xm);
	Mat XmVector2 = ExtractSubVector(CurrentPos, Xm2);
	Mat XmVector3 = ExtractSubVector(CurrentPos, Xm3);
	Mat SVector1 = ExtractSubVector(CurrentPos, S);
	Mat SVector2 = ExtractSubVector(CurrentPos, S2);
	Mat SVector3 = ExtractSubVector(CurrentPos, S3);
	Mat lotaVector1 = ExtractSubVector(CurrentPos, lota);
	Mat lotaVector2 = ExtractSubVector(CurrentPos, lota2);
	Mat lotaVector3 = ExtractSubVector(CurrentPos, lota3);

	int N = JointCount;
	//X1
	Mat DeltaFXm1 = Mat::zeros(1, N, CV_32F);
	float gfm_1 = FunctionXm(XmVector1, OriPos.row(0), XmVector2, OriPos.row(1), XmVector3, OriPos.row(2), StartPos, EndPos);
	for (size_t i = 0; i < N; i++)
	{

		float gfm_p_ei = FunctionXm(PosDelta(XmVector1, i, Plus), OriPos.row(0), XmVector2, OriPos.row(1), XmVector3, OriPos.row(2), StartPos, EndPos);
		DeltaFXm1.at<float>(0, i) = ((gfm_p_ei - gfm_1) / deltaX.at<float>(0, i));
	}

	Mat LotaDeltaFXmD1;
	auto Dxm_1 = CalcDJ(XmVector1);
	for (size_t k = 0; k < JointCount; k++)//PosCount
	{
		float DeltaDXmi = 0;

		auto PosDxm = CalcDJ(PosDelta(XmVector1, k, Plus));
		for (size_t i = 0; i < JointCount; i++)
		{
			DeltaDXmi += lotaVector1.at<float>(0, i) * (PosDxm[i] - Dxm_1[i]) / deltaX.at<float>(0, i);
		}
		LotaDeltaFXmD1.push_back(DeltaDXmi);
	}
	//X2
	Mat DeltaFXm2 = Mat::zeros(1, N, CV_32F);
	float gfm_2 = FunctionXm(XmVector1, OriPos.row(0), XmVector2, OriPos.row(1), XmVector3, OriPos.row(2), StartPos, EndPos);
	for (size_t i = 0; i < N; i++)
	{

		float gfm_p_ei = FunctionXm(XmVector1, OriPos.row(0), PosDelta(XmVector2, i, Plus), OriPos.row(1), XmVector3, OriPos.row(2), StartPos, EndPos);
		DeltaFXm2.at<float>(0, i) = ((gfm_p_ei - gfm_2) / deltaX.at<float>(0, i));
	}

	Mat LotaDeltaFXmD2;
	auto Dxm_2 = CalcDJ(XmVector2);
	for (size_t k = 0; k < JointCount; k++)//PosCount
	{
		float DeltaDXmi = 0;

		auto PosDxm = CalcDJ(PosDelta(XmVector2, k, Plus));
		for (size_t i = 0; i < JointCount; i++)
		{
			DeltaDXmi += lotaVector2.at<float>(0, i) * (PosDxm[i] - Dxm_2[i]) / deltaX.at<float>(0, i);
		}
		LotaDeltaFXmD2.push_back(DeltaDXmi);
	}
	//X3
	Mat DeltaFXm3 = Mat::zeros(1, N, CV_32F);
	float gfm_3 = FunctionXm(XmVector1, OriPos.row(0), XmVector2, OriPos.row(1), XmVector3, OriPos.row(2), StartPos, EndPos);
	for (size_t i = 0; i < N; i++)
	{
		float gfm_p_ei = FunctionXm(XmVector1, OriPos.row(0), XmVector2, OriPos.row(1), PosDelta(XmVector3, i, Plus), OriPos.row(2), StartPos, EndPos);
		DeltaFXm3.at<float>(0, i) = ((gfm_p_ei - gfm_3) / deltaX.at<float>(0, i));
	}

	Mat LotaDeltaFXmD3;
	auto Dxm_3 = CalcDJ(XmVector3);
	for (size_t k = 0; k < JointCount; k++)//PosCount
	{
		float DeltaDXmi = 0;

		auto PosDxm = CalcDJ(PosDelta(XmVector3, k, Plus));
		for (size_t i = 0; i < JointCount; i++)
		{
			DeltaDXmi += lotaVector3.at<float>(0, i) * (PosDxm[i] - Dxm_3[i]) / deltaX.at<float>(0, i);
		}
		LotaDeltaFXmD3.push_back(DeltaDXmi);
	}

	Mat Gradient_Xm1 = DeltaFXm1 + LotaDeltaFXmD1.t(); /*+ muVector - tauVector*/
	Mat Gradient_Xm2 = DeltaFXm2 + LotaDeltaFXmD2.t();
	Mat Gradient_Xm3 = DeltaFXm3 + LotaDeltaFXmD3.t();
	Mat Gradient_L1 = Get_gk(XmVector1, SVector1);
	Mat Gradient_L2 = Get_gk(XmVector2, SVector2);
	Mat Gradient_L3 = Get_gk(XmVector3, SVector3);
	Mat Gradient_S1 = 2 * lotaVector1.mul(SVector1);
	Mat Gradient_S2 = 2 * lotaVector2.mul(SVector2);
	Mat Gradient_S3 = 2 * lotaVector3.mul(SVector3);

	Gradient.push_back(Gradient_Xm1.t());
	Gradient.push_back(Gradient_Xm2.t());
	Gradient.push_back(Gradient_Xm3.t());
	Gradient.push_back(Gradient_L1);
	Gradient.push_back(Gradient_L2);
	Gradient.push_back(Gradient_L3);
	Gradient.push_back(Gradient_S1.t());
	Gradient.push_back(Gradient_S2.t());
	Gradient.push_back(Gradient_S3.t());
	return Gradient;
}

//計算牛頓法中的 海森矩陣，用於路徑優化過程。
Mat NewtonOptimize::CalcHessian(cv::Mat CurrentPos, cv::Mat OriPos1, cv::Mat OriPos2, cv::Mat OriPos3, cv::Mat StartPos, cv::Mat EndPos) {
	int P = CubeCount;
	int N = JointCount;
	CurrentPos = CurrentPos.t();
	Mat XmVector1 = ExtractSubVector(CurrentPos, Xm);
	Mat XmVector2 = ExtractSubVector(CurrentPos, Xm2);
	Mat XmVector3 = ExtractSubVector(CurrentPos, Xm3);
	Mat SVector1 = ExtractSubVector(CurrentPos, S);
	Mat SVector2 = ExtractSubVector(CurrentPos, S2);
	Mat SVector3 = ExtractSubVector(CurrentPos, S3);
	Mat lotaVector1 = ExtractSubVector(CurrentPos, lota);
	Mat lotaVector2 = ExtractSubVector(CurrentPos, lota2);
	Mat lotaVector3 = ExtractSubVector(CurrentPos, lota3);

	Mat Mu_Tau = Mat::zeros(N, N, CV_32F);
	Mat L_Xm1 = Mat::zeros(N, N, CV_32F);
	//chrono::high_resolution_clock::time_point J1 = chrono::high_resolution_clock::now();
	for (size_t i = 0; i < N; i++)
	{
		float fm = FunctionXm(XmVector1, OriPos1, XmVector2, OriPos2, XmVector3, OriPos3, StartPos, EndPos);
		float fm_p_ei = FunctionXm(PosDelta(XmVector1, i, Plus), OriPos1, XmVector2, OriPos2, XmVector3, OriPos3, StartPos, EndPos);
		float fm_m_ei = FunctionXm(PosDelta(XmVector1, i, Minus), OriPos1, XmVector2, OriPos2, XmVector3, OriPos3, StartPos, EndPos);
		vector<float> Dm = CalcDJ(XmVector1);
		vector<float> Dm_p_ei = CalcDJ(PosDelta(XmVector1, i, Plus));
		vector<float> Dm_m_ei = CalcDJ(PosDelta(XmVector1, i, Minus));
		for (size_t j = 0; j < N; j++)
		{
			float fm_p_ej = FunctionXm(PosDelta(XmVector1, j, Plus), OriPos1, XmVector2, OriPos2, XmVector3, OriPos3, StartPos, EndPos);
			float fm_p_ei_ej = FunctionXm(PosDelta(XmVector1, i, j), OriPos1, XmVector2, OriPos2, XmVector3, OriPos3, StartPos, EndPos);
			float Dm_Xmi_Xmj = 0;
			if (i == j) {  // i == j
				for (size_t k = 0; k < N; k++) {
					Dm_Xmi_Xmj += lotaVector1.at<float>(0, k) * ((Dm_p_ei[k] - 2 * Dm[k] + Dm_m_ei[k]) / pow(Delta, 2));
				}
				L_Xm1.at<float>(i, j) = ((fm_p_ei - 2 * fm + fm_m_ei) / pow(Delta, 2)) + Dm_Xmi_Xmj;
			}
			else {
				vector<float> Dm_p_ej = CalcDJ(PosDelta(XmVector1, j, Plus));
				vector<float> Dm_p_ei_ej = CalcDJ(PosDelta(XmVector1, i, j));
				for (size_t k = 0; k < N; k++) {
					Dm_Xmi_Xmj += lotaVector1.at<float>(0, k) * ((Dm_p_ei_ej[k] - Dm_p_ei[k] - Dm_p_ej[k] + Dm[k]) / pow(Delta, 2));
				}
				L_Xm1.at<float>(i, j) = (fm_p_ei_ej - fm_p_ei - fm_p_ej + fm) / pow(Delta, 2) + Dm_Xmi_Xmj;
			}
		}
	}

	//X2
	Mat L_Xm2 = Mat::zeros(N, N, CV_32F);
	for (size_t i = 0; i < N; i++)
	{
		float fm = FunctionXm(XmVector1, OriPos1, XmVector2, OriPos2, XmVector3, OriPos3, StartPos, EndPos);
		float fm_p_ei = FunctionXm(XmVector1, OriPos1, PosDelta(XmVector2, i, Plus), OriPos2, XmVector3, OriPos3, StartPos, EndPos);
		float fm_m_ei = FunctionXm(XmVector1, OriPos1, PosDelta(XmVector2, i, Minus), OriPos2, XmVector3, OriPos3, StartPos, EndPos);
		vector<float> Dm = CalcDJ(XmVector2);
		vector<float> Dm_p_ei = CalcDJ(PosDelta(XmVector2, i, Plus));
		vector<float> Dm_m_ei = CalcDJ(PosDelta(XmVector2, i, Minus));
		for (size_t j = 0; j < N; j++)
		{
			float fm_p_ej = FunctionXm(XmVector1, OriPos1, PosDelta(XmVector2, j, Plus), OriPos2, XmVector3, OriPos3, StartPos, EndPos);
			float fm_p_ei_ej = FunctionXm(XmVector1, OriPos1, PosDelta(XmVector2, i, j), OriPos2, XmVector3, OriPos3, StartPos, EndPos);
			float Dm_Xmi_Xmj = 0;
			if (i == j) {  // i == j
				for (size_t k = 0; k < N; k++) {
					Dm_Xmi_Xmj += lotaVector2.at<float>(0, k) * ((Dm_p_ei[k] - 2 * Dm[k] + Dm_m_ei[k]) / pow(Delta, 2));
				}
				L_Xm2.at<float>(i, j) = ((fm_p_ei - 2 * fm + fm_m_ei) / pow(Delta, 2)) + Dm_Xmi_Xmj;
			}
			else {
				vector<float> Dm_p_ej = CalcDJ(PosDelta(XmVector2, j, Plus));
				vector<float> Dm_p_ei_ej = CalcDJ(PosDelta(XmVector2, i, j));
				for (size_t k = 0; k < N; k++) {
					Dm_Xmi_Xmj += lotaVector2.at<float>(0, k) * ((Dm_p_ei_ej[k] - Dm_p_ei[k] - Dm_p_ej[k] + Dm[k]) / pow(Delta, 2));
				}
				L_Xm2.at<float>(i, j) = (fm_p_ei_ej - fm_p_ei - fm_p_ej + fm) / pow(Delta, 2) + Dm_Xmi_Xmj;
			}
		}
	}
	//X3
	Mat L_Xm3 = Mat::zeros(N, N, CV_32F);
	for (size_t i = 0; i < N; i++)
	{
		float fm = FunctionXm(XmVector1, OriPos1, XmVector2, OriPos2, XmVector3, OriPos3, StartPos, EndPos);
		float fm_p_ei = FunctionXm(XmVector1, OriPos1, XmVector2, OriPos2, PosDelta(XmVector3, i, Plus), OriPos3, StartPos, EndPos);
		float fm_m_ei = FunctionXm(XmVector1, OriPos1, XmVector2, OriPos2, PosDelta(XmVector3, i, Minus), OriPos3, StartPos, EndPos);
		vector<float> Dm = CalcDJ(XmVector3);
		vector<float> Dm_p_ei = CalcDJ(PosDelta(XmVector3, i, Plus));
		vector<float> Dm_m_ei = CalcDJ(PosDelta(XmVector3, i, Minus));
		for (size_t j = 0; j < N; j++)
		{
			float fm_p_ej = FunctionXm(XmVector1, OriPos1, XmVector2, OriPos2, PosDelta(XmVector3, j, Plus), OriPos3, StartPos, EndPos);
			float fm_p_ei_ej = FunctionXm(XmVector1, OriPos1, XmVector2, OriPos2, PosDelta(XmVector3, i, j), OriPos3, StartPos, EndPos);
			float Dm_Xmi_Xmj = 0;
			if (i == j) {  // i == j
				for (size_t k = 0; k < N; k++) {
					Dm_Xmi_Xmj += lotaVector3.at<float>(0, k) * ((Dm_p_ei[k] - 2 * Dm[k] + Dm_m_ei[k]) / pow(Delta, 2));
				}
				L_Xm3.at<float>(i, j) = ((fm_p_ei - 2 * fm + fm_m_ei) / pow(Delta, 2)) + Dm_Xmi_Xmj;
			}
			else {
				vector<float> Dm_p_ej = CalcDJ(PosDelta(XmVector3, j, Plus));
				vector<float> Dm_p_ei_ej = CalcDJ(PosDelta(XmVector3, i, j));
				for (size_t k = 0; k < N; k++) {
					Dm_Xmi_Xmj += lotaVector3.at<float>(0, k) * ((Dm_p_ei_ej[k] - Dm_p_ei[k] - Dm_p_ej[k] + Dm[k]) / pow(Delta, 2));
				}
				L_Xm3.at<float>(i, j) = (fm_p_ei_ej - fm_p_ei - fm_p_ej + fm) / pow(Delta, 2) + Dm_Xmi_Xmj;
			}
		}
	}
	//	chrono::high_resolution_clock::time_point J2 = chrono::high_resolution_clock::now();
	//std::cout << chrono::duration_cast<chrono::milliseconds>(J2 - J1).count() << " ms" << endl;
	Mat X1_X2 = Mat::zeros(N, N, CV_32F);
	for (int i = 0; i < JointCount; i++) {
		float fm = FunctionXm(XmVector1, OriPos1, XmVector2, OriPos2, XmVector3, OriPos3, StartPos, EndPos);
		float fm_p_ei = FunctionXm(PosDelta(XmVector1, i, Plus), OriPos1, XmVector2, OriPos2, XmVector3, OriPos3, StartPos, EndPos);
		for (int j = 0; j < JointCount; j++) {
			float fm_p_ej = FunctionXm(XmVector1, OriPos1, PosDelta(XmVector2, j, Plus), OriPos2, XmVector3, OriPos3, StartPos, EndPos);
			float fm_p_ei_ej = FunctionXm(PosDelta(XmVector1, i, Plus), OriPos1, PosDelta(XmVector2, j, Plus), OriPos2, XmVector3, OriPos3, StartPos, EndPos);
			X1_X2.at<float>(i, j) = (fm_p_ei_ej - fm_p_ei - fm_p_ej + fm) / pow(Delta, 2);
		}
	}

	Mat X2_X3 = Mat::zeros(N, N, CV_32F);
	for (int i = 0; i < JointCount; i++) {
		float fm = FunctionXm(XmVector1, OriPos1, XmVector2, OriPos2, XmVector3, OriPos3, StartPos, EndPos);
		float fm_p_ei = FunctionXm(XmVector1, OriPos1, PosDelta(XmVector2, i, Plus), OriPos2, XmVector3, OriPos3, StartPos, EndPos);
		for (int j = 0; j < JointCount; j++) {
			float fm_p_ej = FunctionXm(XmVector1, OriPos1, XmVector2, OriPos2, PosDelta(XmVector3, j, Plus), OriPos3, StartPos, EndPos);
			float fm_p_ei_ej = FunctionXm(XmVector1, OriPos1, PosDelta(XmVector2, i, Plus), OriPos2, PosDelta(XmVector3, j, Plus), OriPos3, StartPos, EndPos);
			X2_X3.at<float>(i, j) = (fm_p_ei_ej - fm_p_ei - fm_p_ej + fm) / pow(Delta, 2);
		}
	}
	//x1////////////////////////////////////////////////
	Mat L_Xm_lota1 = Mat::zeros(N, N, CV_32F);
	for (size_t i = 0; i < N; i++)
	{
		vector<float> Dm = CalcDJ(XmVector1);
		vector<float> Dm_p_ei = CalcDJ(PosDelta(XmVector1, i, Plus));
		for (size_t j = 0; j < N; j++)
		{
			L_Xm_lota1.at<float>(i, j) = (Dm_p_ei[j] - Dm[j]) / Delta;//   l54
		}
	}
	//X2
	Mat L_Xm_lota2 = Mat::zeros(N, N, CV_32F);
	for (size_t i = 0; i < N; i++)
	{
		vector<float> Dm = CalcDJ(XmVector2);
		vector<float> Dm_p_ei = CalcDJ(PosDelta(XmVector2, i, Plus));
		for (size_t j = 0; j < N; j++)
		{
			L_Xm_lota2.at<float>(i, j) = (Dm_p_ei[j] - Dm[j]) / Delta;//   l54
		}
	}
	//X3
	Mat L_Xm_lota3 = Mat::zeros(N, N, CV_32F);
	for (size_t i = 0; i < N; i++)
	{
		vector<float> Dm = CalcDJ(XmVector3);
		vector<float> Dm_p_ei = CalcDJ(PosDelta(XmVector3, i, Plus));
		for (size_t j = 0; j < N; j++)
		{
			L_Xm_lota3.at<float>(i, j) = (Dm_p_ei[j] - Dm[j]) / Delta;//   l54
		}
	}

	Mat L_lota_S1 = Mat::zeros(N, N, CV_32F);
	for (size_t i = 0; i < N; i++)
	{
		L_lota_S1.at<float>(i, i) = 2 * SVector1.at<float>(0, i);
	}
	Mat L_lota_S2 = Mat::zeros(N, N, CV_32F);
	for (size_t i = 0; i < N; i++)
	{
		L_lota_S2.at<float>(i, i) = 2 * SVector2.at<float>(0, i);
	}
	Mat L_lota_S3 = Mat::zeros(N, N, CV_32F);
	for (size_t i = 0; i < N; i++)
	{
		L_lota_S3.at<float>(i, i) = 2 * SVector3.at<float>(0, i);
	}
	Mat L_S1 = Mat::zeros(N, N, CV_32F);
	for (size_t i = 0; i < N; i++)
	{
		L_S1.at<float>(i, i) = 2 * lotaVector1.at<float>(0, i);
	}
	Mat L_S2 = Mat::zeros(N, N, CV_32F);
	for (size_t i = 0; i < N; i++)
	{
		L_S2.at<float>(i, i) = 2 * lotaVector2.at<float>(0, i);
	}
	Mat L_S3 = Mat::zeros(N, N, CV_32F);
	for (size_t i = 0; i < N; i++)
	{
		L_S3.at<float>(i, i) = 2 * lotaVector3.at<float>(0, i);
	}

	Mat H = Mat::zeros(9 * N, 9 * N, CV_32F);
	L_Xm1.copyTo(H(Rect(Point(0, 0), L_Xm1.size())));
	X1_X2.copyTo(H(Rect(Point(N, 0), X1_X2.size())));
	L_Xm2.copyTo(H(Rect(Point(N, N), L_Xm2.size())));
	X2_X3.copyTo(H(Rect(Point(2 * N, N), X2_X3.size())));
	L_Xm3.copyTo(H(Rect(Point(2 * N, 2 * N), L_Xm3.size())));
	L_Xm_lota1.copyTo(H(Rect(Point(3 * N, 0), L_Xm_lota1.size())));
	L_Xm_lota2.copyTo(H(Rect(Point(4 * N, N), L_Xm_lota2.size())));
	L_Xm_lota3.copyTo(H(Rect(Point(5 * N, 2 * N), L_Xm_lota3.size())));
	L_lota_S1.copyTo(H(Rect(Point(6 * N, 3 * N), L_lota_S1.size())));
	L_lota_S2.copyTo(H(Rect(Point(7 * N, 4 * N), L_lota_S1.size())));
	L_lota_S3.copyTo(H(Rect(Point(8 * N, 5 * N), L_lota_S1.size())));
	L_S1.copyTo(H(Rect(Point(6 * N, 6 * N), L_S1.size())));
	L_S2.copyTo(H(Rect(Point(7 * N, 7 * N), L_S1.size())));
	L_S3.copyTo(H(Rect(Point(8 * N, 8 * N), L_S1.size())));
	//L_mu_U.copyTo(H(Rect(Point(2 * N + 2 * P, N + 2 * P), L_mu_U.size())));
	//L_U.copyTo(H(Rect(Point(2 * N + 2 * P, 2 * N + 2 * P), L_U.size())));
	//L_tau_V.copyTo(H(Rect(Point(4 * N + 2 * P, 3 * N + 2 * P), L_tau_V.size())));
	//L_V.copyTo(H(Rect(Point(4 * N + 2 * P, 4 * N + 2 * P), L_V.size())));
	completeSymm(H);
	return H;
}

//建立一個三維工作空間（WorkSpace），並將該空間分割為若干小的立方體區域，用於後續的碰撞檢測或路徑規劃。
//程式碼通過離散化工作空間，將每個點的坐標計算出來並存儲，還生成了立方體的邊界點和中心點來進行檢測。
void NewtonOptimize::MakeWorkSpace(void) {
	for (long z = 0; z < LWH_SIZE_Z; z++) {
		for (long y = 0; y < LWH_SIZE_XY; y++) {
			for (long x = 0; x < LWH_SIZE_XY; x++) {
				WS[z][y][x].x = (LWH_XY_MIN + (x * RESOLUTION));
				WS[z][y][x].y = (LWH_XY_MIN + (y * RESOLUTION));
				WS[z][y][x].z = (LWH_Z_MIN + (z * RESOLUTION));
			}
		}
	}
	for (long z = 0; z < LWH_SIZE_Z - 1; z++) {
		for (long y = 0; y < LWH_SIZE_XY - 1; y++) {
			for (long x = 0; x < LWH_SIZE_XY - 1; x++) {
				for (int k = 0; k < 8; k++) {
					box[z][y][x][0] = WS[z][y][x];
					box[z][y][x][1] = WS[z][y][x + 1];
					box[z][y][x][2] = WS[z][y + 1][x + 1];
					box[z][y][x][3] = WS[z][y + 1][x];
					box[z][y][x][4] = WS[z + 1][y][x];
					box[z][y][x][5] = WS[z + 1][y][x + 1];
					box[z][y][x][6] = WS[z + 1][y + 1][x + 1];
					box[z][y][x][7] = WS[z + 1][y + 1][x];

					/* dectectPt(i,j,k,1,:)=[(WS(i,j,k,:)+WS(i+1,j+1,k+1,:))./2];%     I */
					dectectPt[z][y][x].x = ((float)(WS[z][y][x].x + WS[z + 1][y + 1][x + 1].x)) / 2;
					dectectPt[z][y][x].y = ((float)(WS[z][y][x].y + WS[z + 1][y + 1][x + 1].y)) / 2;
					dectectPt[z][y][x].z = ((float)(WS[z][y][x].z + WS[z + 1][y + 1][x + 1].z)) / 2;
				}
			}
		}
	}
}

//機械手臂的正向運動學 (Forward Kinematics)
//適用於機械手臂的路徑規劃或運動控制，在輸入關節角度後，可以計算出機械手臂末端的實際位置
std::vector<float> NewtonOptimize::forwardKinematic(cv::Mat PathData) {
	static const float	r[] = { 428.31,172.348,252.816,145.719,118.029,0, };
	float	    Resolution = sqrt(((RESOLUTION / 2) * (RESOLUTION / 2)) * 3);
	TArray4X4	tmp, tmpC;
	TArray2X4	link[7];

	float		sj[CountOfArray(link)];
	bool		sj01[CountOfArray(sj)];
	float       C[6];
	vector<string> row;
	//string line, word;
	//auto output = std::vector<std::vector<std::vector<std::vector<float>>>>(PathData.rows, std::vector<std::vector<std::vector<float>>>(CountOfArray(box), std::vector<std::vector<float>>(CountOfArray(box[0]), std::vector<float>(CountOfArray(box[0][0])))));

	for (int j = 0; j < PathData.cols; j++)
	{
		C[j] = PathData.at<float>(0, j);
	}
	float	J[] =
	{
		float(((C[0]) / 180.0) * M_PI),
		float(((C[1]) / 180.0) * M_PI),
		float(((C[2]) / 180.0) * M_PI),
		float(((C[3]) / 180.0) * M_PI),
		float(((C[4]) / 180.0) * M_PI),
		float(((C[5]) / 180.0) * M_PI),
	};

	TArray2X4	link0 =
	{
		{ 0, 0, 0, 1, },
		{ 0, 0, 0, 1, },
	};

	// L1
	TArray2X4	link1 =
	{
		{ 0, 0, 0, 1, },
		{ 0, 0, 746.81, 1, },
	};

	TArray4X4	C1 =
	{
		{ cos(J[0]), -sin(J[0]), 0, 0, },
		{ sin(J[0]), cos(J[0]), 0, 0, },
		{ 0, 0, 1, 0, },
		{ 0, 0, 0, 1, },
	};

	TArray4X4	T10 =
	{
		{ 1, 0, 0, 0, },
		{ 0, 1, 0, 0, },
		{ 0, 0, 1, 0,},//200
		{ 0, 0, 0, 1, },
	};

	// L2
	TArray2X4		link2 =
	{
		{ 0, 0, 0, 1, },
		{ 0, 0, 1152.263, 1, },
	};

	TArray4X4		C2 =
	{
		{ cos(J[1]), -sin(J[1]), 0, 0, },
		{ sin(J[1]), cos(J[1]), 0, 0, },
		{ 0, 0, 1, 0, },
		{ 0, 0, 0, 1, },
	};

	TArray4X4		T21 =
	{
		{ 1, 0, 0, 175, },
		{ 0, 1, 0, -428.31, },
		{ 0, 0, 1, 516.595,},
		{ 0, 0, 0, 1, },
	};

	// L3
	TArray2X4		link3 =
	{
		{0, 0, 0, 1, },
		{ 0, 0, 571.044, 1, },
	};

	TArray4X4		C3 =
	{
		{ cos(J[2]), -sin(J[2]), 0, 0, },
		{ sin(J[2]), cos(J[2]), 0, 0, },
		{ 0, 0, 1, 0, },
		{ 0, 0, 0, 1, },
	};

	TArray4X4		T32 =
	{
		{ 1, 0, 0, 0, },
		{ 0, 0, -1, -172.348, },
		{ 0, 1, 0, 1152.263,},
		{ 0, 0, 0, 1, },
	};

	// L4
	TArray2X4		link4 =
	{
		{ 0, 0, 0, 1, },
		{ 0, 0, 861.908, 1, },
	};

	TArray4X4		C4 =
	{
		{ cos(J[3]), -sin(J[3]), 0, 0, },
		{ sin(J[3]), cos(J[3]), 0, 0, },
		{ 0, 0, 1, 0, },
		{ 0, 0, 0, 1, },
	};

	TArray4X4		T43 =
	{
		{ 0, 0, 1, 145.719, },
		{ 0, -1, 0, -50, },
		{ 1, 0, 0, 255.962,},
		{ 0, 0, 0, 1, },
	};

	// L5
	TArray2X4		link5 =
	{
		{ 0, 0, 0, 1, },
		{ 0, 0, 334.494, 1, },
	};

	TArray4X4		C5 =
	{
		{ cos(J[4]), -sin(J[4]), 0, 0, },
		{ sin(J[4]), cos(J[4]), 0, 0, },
		{ 0, 0, 1, 0, },
		{ 0, 0, 0, 1, },
	};

	TArray4X4		T54 =
	{
		{ 0, 0, -1, 145.719, },
		{ 1, 0, 0, 1036.908, },
		{ 0, -1, 1, -175,},
		{ 0, 0, 0, 1, },
	};

	// L6
	TArray2X4		link6 =
	{
		{ 0,0, 0, 1, },
		{ 0,0, 0, 1, },
	};

	TArray4X4		C6 =
	{
		{ cos(J[5]), -sin(J[5]), 0, 0, },
		{ sin(J[5]), cos(J[5]), 0, 0, },
		{ 0, 0, 1, 0, },
		{ 0, 0, 0, 1, },

	};

	TArray4X4		T65 =
	{
		{ 1, 0, 0, 0, },
		{ 0, 1, 0, 0, },
		{ 0, 0, 1, 0,},
		{ 0, 0, 0, 1, },
	};

	/* Link0 */

	ArrayMul(T10, C1, tmpC);
	ArrayMul(tmpC, link1, link1);
	/* Link2 */
	ArrayMul(T21, C2, tmp);
	ArrayMul(tmpC, tmp, tmpC);
	ArrayMul(tmpC, link2, link2);
	/* Link3 */
	ArrayMul(T32, C3, tmp);
	ArrayMul(tmpC, tmp, tmpC);
	ArrayMul(tmpC, link3, link3);
	/* Link4 */
	ArrayMul(T43, C4, tmp);
	ArrayMul(tmpC, tmp, tmpC);
	ArrayMul(tmpC, link4, link4);
	/* Link5 */
	ArrayMul(T54, C5, tmp);
	ArrayMul(tmpC, tmp, tmpC);
	ArrayMul(tmpC, link5, link5);
	/* Link6 */
	ArrayMul(T65, C6, tmp);
	ArrayMul(tmpC, tmp, tmpC);
	ArrayMul(tmpC, link6, link6);
	std::vector<float>result;
	for (int i = 0; i < 6; i++) {
		result.push_back(link6[0][i]);
	}
	return result;
}


//這段程式碼的主要目的是模擬機械手臂的運動，並計算機械手臂路徑上每個點的安全分數（sjMax），用於判斷機械手臂是否可能與某個障礙物碰撞。
std::vector<float> NewtonOptimize::RobotArm(cv::Mat PathData, float cubeX, float cubeY, float cubeZ, T3DValue point)
{
	static const float	r[] = { 428.31,172.348,252.816,145.719,118.029,0, };
	float	    Resolution = sqrt(((RESOLUTION / 2) * (RESOLUTION / 2)) * 3);
	TArray4X4	tmp, tmpC;
	TArray2X4	link[7];

	float		sj[CountOfArray(link)];
	bool		sj01[CountOfArray(sj)];
	float       C[6];
	vector<string> row;
	//string line, word;
	auto output = std::vector<float>(PathData.rows);

	for (int f = 0; f < PathData.rows; f++)
	{
		for (int j = 0; j < PathData.row(f).cols; j++)
		{
			C[j] = PathData.at<float>(f, j);
		}
		float	J[] =
		{
			float(((C[0]) / 180.0) * M_PI),
			float(((C[1]) / 180.0) * M_PI),
			float(((C[2]) / 180.0) * M_PI),
			float(((C[3]) / 180.0) * M_PI),
			float(((C[4]) / 180.0) * M_PI),
			float(((C[5]) / 180.0) * M_PI),
		};

		TArray2X4	link0 =//000
		{
			{ 0, 0, 0, 1, },
			{ 0, 0, 200, 1, },
		};

		// L1
		TArray2X4	link1 =
		{
			{ 0, 0, 0, 1, },
			{ 0, 0, 746.81, 1, },
		};

		TArray4X4	C1 =
		{
			{ cos(J[0]), -sin(J[0]), 0, 0, },
			{ sin(J[0]), cos(J[0]), 0, 0, },
			{ 0, 0, 1, 0, },
			{ 0, 0, 0, 1, },
		};

		TArray4X4	T10 =
		{
			{ 1, 0, 0, 0, },
			{ 0, 1, 0, 0, },
			{ 0, 0, 1, 0,},
			{ 0, 0, 0, 1, },
		};

		// L2
		TArray2X4		link2 =
		{
			{ 0, 0, 0, 1, },
			{ 0, 0, 1152.263, 1, },
		};

		TArray4X4		C2 =//000
		{
			{1, 0, 0, 0, },
			{ 0, cos(J[1]), -sin(J[1]), 0, },
			{ 0, sin(J[1]), cos(J[1]), 0, },
			{ 0, 0, 0, 1, },
		};

		TArray4X4		T21 =
		{
			{ 1, 0, 0, 175, },
			{ 0, 1, 0, -428.31, },
			{ 0, 0, 1, 516.595,},
			{ 0, 0, 0, 1, },
		};

		// L3
		TArray2X4		link3 =
		{
			{0, 0, 0, 1, },
			{ 0, 0, 571.044, 1, },
		};

		TArray4X4		C3 =
		{
			{cos(J[2]), -sin(J[2]), 0, 0, },
			{sin(J[2]), cos(J[2]), 0, 0, },
			{ 0, 0, 1, 0, },
			{ 0, 0, 0, 1, },
		};

		TArray4X4		T32 =
		{
			{ 1, 0, 0, 0, },
			{ 0, 0, -1, -172.348, },
			{ 0, 1, 0, 1152.263,},
			{ 0, 0, 0, 1, },
		};

		// L4
		TArray2X4		link4 =
		{
			{ 0, 0, 0, 1, },
			{ 0, 0, 861.908, 1, },
		};

		TArray4X4		C4 =
		{
			{ cos(J[3]), -sin(J[3]), 0, 0, },
			{ sin(J[3]), cos(J[3]), 0, 0, },
			{ 0, 0, 1, 0, },
			{ 0, 0, 0, 1, },
		};

		TArray4X4		T43 =
		{
			{ 0, 0, 1, 145.719, },
			{ 0, -1, 0, -50, },
			{ 1, 0, 0, 255.962,},
			{ 0, 0, 0, 1, },
		};

		// L5
		TArray2X4		link5 =
		{
			{ 0, 0, 0, 1, },
			{ 0, 0, 334.494, 1, },
		};

		TArray4X4		C5 =
		{
			{cos(J[4]), -sin(J[4]), 0, 0, },
			{sin(J[4]), cos(J[4]), 0, 0, },
			{ 0, 0, 1, 0, },
			{ 0, 0, 0, 1, },
		};

		TArray4X4		T54 =
		{
			{ 0, 0, -1, 145.719, },
			{ 1, 0, 0, 1036.908, },
			{ 0, -1, 1, -175,},
			{ 0, 0, 0, 1, },
		};

		// L6
		TArray2X4		link6 =
		{
			{ 0,0, 0, 1, },
			{ 0,0, 0, 1, },
		};

		TArray4X4		C6 =
		{
			{cos(J[5]), -sin(J[5]), 0, 0, },
			{sin(J[5]), cos(J[5]), 0, 0, },
			{ 0, 0, 1, 0, },
			{ 0, 0, 0, 1, },

		};

		TArray4X4		T65 =
		{
			{ 1, 0, 0, 0, },
			{ 0, 1, 0, 0, },
			{ 0, 0, 1, 0,},
			{ 0, 0, 0, 1, },
		};


		/* Link0 */

		ArrayMul(T10, C1, tmpC);
		ArrayMul(tmpC, link1, link1);
		/* Link2 */
		ArrayMul(T21, C2, tmp);
		ArrayMul(tmpC, tmp, tmpC);
		ArrayMul(tmpC, link2, link2);
		/* Link3 */
		ArrayMul(T32, C3, tmp);
		ArrayMul(tmpC, tmp, tmpC);
		ArrayMul(tmpC, link3, link3);
		/* Link4 */
		ArrayMul(T43, C4, tmp);
		ArrayMul(tmpC, tmp, tmpC);
		ArrayMul(tmpC, link4, link4);
		/* Link5 */
		ArrayMul(T54, C5, tmp);
		ArrayMul(tmpC, tmp, tmpC);
		ArrayMul(tmpC, link5, link5);
		/* Link6 */
		ArrayMul(T65, C6, tmp);
		ArrayMul(tmpC, tmp, tmpC);
		ArrayMul(tmpC, link6, link6);

		memcpy(&link[0], &link0, sizeof(link0));
		memcpy(&link[1], &link1, sizeof(link1));
		memcpy(&link[2], &link2, sizeof(link2));
		memcpy(&link[3], &link3, sizeof(link3));
		memcpy(&link[4], &link4, sizeof(link4));
		memcpy(&link[5], &link5, sizeof(link5));
		memcpy(&link[6], &link6, sizeof(link6));

		float       sjMax = 0.0;
		for (int cnt = 0; cnt < CountOfArray(sj); cnt++) {
			sj[cnt] = 0.0;
			sj01[cnt] = false;
		}

		for (int a = 0; a < CountOfArray(link); a++) {
			T3DValue    P0, P1;
			T3DValue	V0 = { cubeX,cubeY,cubeZ };
			float		d;
			float		od, odTmp;
			P1.x = link[a][1][0], P1.y = link[a][1][1], P1.z = link[a][1][2];

			P0.x = link[a][0][0], P0.y = link[a][0][1], P0.z = link[a][0][2];

			//V0 = dectectPt[cubeZ][cubeY][cubeX];
			//V0 = point;

			d = Norm(Cross(Sub(P1, P0), Sub(V0, P0))) / Norm(Sub(P1, P0));
			/*  */
			od = -100000;

			if (d <= (Resolution + r[a])) {

				if (odTmp = ((Dot(Sub(V0, P1), Sub(P0, P1)) / Norm(Sub(P1, P0))) - Norm(Sub(P1, P0))), odTmp > 0) {

					od = odTmp;

				}
				else if (odTmp = ((Dot(Sub(V0, P0), Sub(P1, P0)) / Norm(Sub(P1, P0))) - Norm(Sub(P1, P0))), odTmp > 0) {

					od = odTmp;
				}

				/*  */
				if (d <= r[a]
					&& od > Resolution) {
					sj[a] = (Resolution / od) * 0.5;
					sj01[a] = false;
					continue;
				}

				/*  */
				if (d <= r[a]
					&& od <= Resolution
					&& od > 0) {
					float	maxsafe = 1 + ((d / (Resolution + r[a])) * (0.5 - 1));


					sj[a] = maxsafe + (od / (Resolution)) * (0.5 - maxsafe);
					sj01[a] = true;
					continue;
				}
				/*  */
				if (od > 0
					&& d > r[a]
					&& (sqrt(pow((d - r[a]), 2) + pow(od, 2)) > Resolution)) {
					sj[a] = (Resolution / sqrt(pow((d - r[a]), 2) + pow(od, 2))) * 0.5;
					sj01[a] = false;
					continue;
				}

				/*  */
				if (od > 0
					&& d > r[a]
					&& (sqrt(pow((d - r[a]), 2) + pow(od, 2)) <= Resolution)) {
					float	maxsafe = (1 + ((r[a] / (Resolution + r[a])) * (0.5 - 1)));

					sj[a] = maxsafe + (sqrt(pow((d - r[a]), 2) + pow(od, 2)) / (Resolution)) * (0.5 - maxsafe);

					sj01[a] = true;
					continue;
				}

				/*  */
				sj[a] = 1 + ((d / (Resolution + r[a])) * (0.5 - 1));
				sj01[a] = true;

			}
			else {
				/*  */
				sj01[a] = false;

				/*  */
				if (odTmp = ((Dot(Sub(V0, P1), Sub(P0, P1)) / Norm(Sub(P1, P0))) - Norm(Sub(P1, P0))), odTmp > 0) {
					od = odTmp;
				}
				else if (odTmp = ((Dot(Sub(V0, P0), Sub(P1, P0)) / Norm(Sub(P1, P0))) - Norm(Sub(P1, P0))), odTmp > 0) {
					od = odTmp;
				}

				/*  */
				if (od > 0) {

					sj[a] = (Resolution / sqrt(pow((d - r[a]), 2) + pow(od, 2))) * 0.5;

				}
				else {

					sj[a] = (((Resolution) / (d - r[a]))) * 0.5;
				}
			}

		}

		for (int cnt = 0; cnt < CountOfArray(sj); cnt++) {
			if (!cnt) {
				sjMax = sj[0];
			}
			else if (sj[cnt] > sjMax) {
				sjMax = sj[cnt];
			}
		}

		output[f] = sjMax;

	}
	return output;
}


//模擬機械手臂的運動，並計算每個連桿的安全分數。
//通過正向運動學來確定每個連桿的位姿，並根據與障礙物的距離計算是否存在碰撞風險。
std::vector<float> NewtonOptimize::RobotArmJ(cv::Mat PathData, float cubeX, float cubeY, float cubeZ, T3DValue point)
{
	static const double	r[] = { 428.31,172.348,252.816,145.719,118.029,0, };
	float	    Resolution = sqrt(((RESOLUTION / 2) * (RESOLUTION / 2)) * 3);
	TArray4X4	tmp, tmpC;
	TArray2X4	link[7];

	float		sj[CountOfArray(link)];
	bool		sj01[CountOfArray(sj)];
	float       C[6];
	vector<string> row;
	//string line, word;

	vector<float> result;
	for (int f = 0; f < PathData.rows; f++)
	{
		for (int j = 0; j < PathData.row(f).cols; j++)
		{
			C[j] = PathData.at<float>(f, j);
		}
		float	J[] =
		{
			(((C[0]) / 180.0) * M_PI),
			(((C[1]) / 180.0) * M_PI),
			(((C[2]) / 180.0) * M_PI),
			(((C[3]) / 180.0) * M_PI),
			(((C[4]) / 180.0) * M_PI),
			(((C[5]) / 180.0) * M_PI),
		};

		TArray2X4	link0 =
		{
			{ 0, 0, 0, 1, },
			{ 0, 0, 0, 1, },
		};

		// L1
		TArray2X4	link1 =
		{
			{ 0, 0, 0, 1, },
			{ 0, 0, 746.81, 1, },
		};

		TArray4X4	C1 =
		{
			{ cos(J[0]), -sin(J[0]), 0, 0, },
			{ sin(J[0]), cos(J[0]), 0, 0, },
			{ 0, 0, 1, 0, },
			{ 0, 0, 0, 1, },
		};

		TArray4X4	T10 =
		{
			{ 1, 0, 0, 0, },
			{ 0, 1, 0, 0, },
			{ 0, 0, 1, 0,},//200
			{ 0, 0, 0, 1, },
		};

		// L2
		TArray2X4		link2 =
		{
			{ 0, 0, 0, 1, },
			{ 0, 0, 1152.263, 1, },
		};

		TArray4X4		C2 =
		{
			{ cos(J[1]), -sin(J[1]), 0, 0, },
			{ sin(J[1]), cos(J[1]), 0, 0, },
			{ 0, 0, 1, 0, },
			{ 0, 0, 0, 1, },
		};

		TArray4X4		T21 =
		{
			{ 1, 0, 0, 175, },
			{ 0, 1, 0, -428.31, },
			{ 0, 0, 1, 516.595,},
			{ 0, 0, 0, 1, },
		};

		// L3
		TArray2X4		link3 =
		{
			{0, 0, 0, 1, },
			{ 0, 0, 571.044, 1, },
		};

		TArray4X4		C3 =
		{
			{ cos(J[2]), -sin(J[2]), 0, 0, },
			{ sin(J[2]), cos(J[2]), 0, 0, },
			{ 0, 0, 1, 0, },
			{ 0, 0, 0, 1, },
		};

		TArray4X4		T32 =
		{
			{ 1, 0, 0, 0, },
			{ 0, 0, -1, -172.348, },
			{ 0, 1, 0, 1152.263,},
			{ 0, 0, 0, 1, },
		};

		// L4
		TArray2X4		link4 =
		{
			{ 0, 0, 0, 1, },
			{ 0, 0, 861.908, 1, },
		};

		TArray4X4		C4 =
		{
			{ cos(J[3]), -sin(J[3]), 0, 0, },
			{ sin(J[3]), cos(J[3]), 0, 0, },
			{ 0, 0, 1, 0, },
			{ 0, 0, 0, 1, },
		};

		TArray4X4		T43 =
		{
			{ 0, 0, 1, 145.719, },
			{ 0, -1, 0, -50, },
			{ 1, 0, 0, 255.962,},
			{ 0, 0, 0, 1, },
		};

		// L5
		TArray2X4		link5 =
		{
			{ 0, 0, 0, 1, },
			{ 0, 0, 334.494, 1, },
		};

		TArray4X4		C5 =
		{
			{ cos(J[4]), -sin(J[4]), 0, 0, },
			{ sin(J[4]), cos(J[4]), 0, 0, },
			{ 0, 0, 1, 0, },
			{ 0, 0, 0, 1, },
		};

		TArray4X4		T54 =
		{
			{ 0, 0, -1, 145.719, },
			{ 1, 0, 0, 1036.908, },
			{ 0, -1, 1, -175,},
			{ 0, 0, 0, 1, },
		};

		// L6
		TArray2X4		link6 =
		{
			{ 0,0, 0, 1, },
			{ 0,0, 0, 1, },
		};

		TArray4X4		C6 =
		{
			{ cos(J[5]), -sin(J[5]), 0, 0, },
			{ sin(J[5]), cos(J[5]), 0, 0, },
			{ 0, 0, 1, 0, },
			{ 0, 0, 0, 1, },

		};

		TArray4X4		T65 =
		{
			{ 1, 0, 0, 0, },
			{ 0, 1, 0, 0, },
			{ 0, 0, 1, 0,},
			{ 0, 0, 0, 1, },
		};


		/* Link0 */

		ArrayMul(T10, C1, tmpC);
		ArrayMul(tmpC, link1, link1);
		/* Link2 */
		ArrayMul(T21, C2, tmp);
		ArrayMul(tmpC, tmp, tmpC);
		ArrayMul(tmpC, link2, link2);
		/* Link3 */
		ArrayMul(T32, C3, tmp);
		ArrayMul(tmpC, tmp, tmpC);
		ArrayMul(tmpC, link3, link3);
		/* Link4 */
		ArrayMul(T43, C4, tmp);
		ArrayMul(tmpC, tmp, tmpC);
		ArrayMul(tmpC, link4, link4);
		/* Link5 */
		ArrayMul(T54, C5, tmp);
		ArrayMul(tmpC, tmp, tmpC);
		ArrayMul(tmpC, link5, link5);
		/* Link6 */
		ArrayMul(T65, C6, tmp);
		ArrayMul(tmpC, tmp, tmpC);
		ArrayMul(tmpC, link6, link6);

		memcpy(&link[0], &link0, sizeof(link0));
		memcpy(&link[1], &link1, sizeof(link1));
		memcpy(&link[2], &link2, sizeof(link2));
		memcpy(&link[3], &link3, sizeof(link3));
		memcpy(&link[4], &link4, sizeof(link4));
		memcpy(&link[5], &link5, sizeof(link5));
		memcpy(&link[6], &link6, sizeof(link6));

		float       sjMax = 0.0;
		for (int cnt = 0; cnt < CountOfArray(sj); cnt++) {
			sj[cnt] = 0.0;
			sj01[cnt] = false;
		}
		for (int a = 0; a < CountOfArray(link); a++) {
			T3DValue    P0, P1;
			T3DValue	V0 = { cubeX,cubeY,cubeZ };
			float		d;
			float		od, odTmp;
			P1.x = link[a][1][0], P1.y = link[a][1][1], P1.z = link[a][1][2];

			P0.x = link[a][0][0], P0.y = link[a][0][1], P0.z = link[a][0][2];

			//V0 = dectectPt[cubeZ][cubeY][cubeX];


			d = Norm(Cross(Sub(P1, P0), Sub(V0, P0))) / Norm(Sub(P1, P0));
			/*  */
			od = -100000;

			if (d <= (Resolution + r[a])) {

				if (odTmp = ((Dot(Sub(V0, P1), Sub(P0, P1)) / Norm(Sub(P1, P0))) - Norm(Sub(P1, P0))), odTmp > 0) {

					od = odTmp;

				}
				else if (odTmp = ((Dot(Sub(V0, P0), Sub(P1, P0)) / Norm(Sub(P1, P0))) - Norm(Sub(P1, P0))), odTmp > 0) {

					od = odTmp;
				}

				/*  */
				if (d <= r[a]
					&& od > Resolution) {
					sj[a] = (Resolution / od) * 0.5;
					sj01[a] = false;
					continue;
				}

				/*  */
				if (d <= r[a]
					&& od <= Resolution
					&& od > 0) {
					float	maxsafe = 1 + ((d / (Resolution + r[a])) * (0.5 - 1));


					sj[a] = maxsafe + (od / (Resolution)) * (0.5 - maxsafe);
					sj01[a] = true;
					continue;
				}
				/*  */
				if (od > 0
					&& d > r[a]
					&& (sqrt(pow((d - r[a]), 2) + pow(od, 2)) > Resolution)) {
					sj[a] = (Resolution / sqrt(pow((d - r[a]), 2) + pow(od, 2))) * 0.5;
					sj01[a] = false;
					continue;
				}

				/*  */
				if (od > 0
					&& d > r[a]
					&& (sqrt(pow((d - r[a]), 2) + pow(od, 2)) <= Resolution)) {
					float	maxsafe = (1 + ((r[a] / (Resolution + r[a])) * (0.5 - 1)));

					sj[a] = maxsafe + (sqrt(pow((d - r[a]), 2) + pow(od, 2)) / (Resolution)) * (0.5 - maxsafe);

					sj01[a] = true;
					continue;
				}

				/*  */
				sj[a] = 1 + ((d / (Resolution + r[a])) * (0.5 - 1));
				sj01[a] = true;

			}
			else {
				/*  */
				sj01[a] = false;

				/*  */
				if (odTmp = ((Dot(Sub(V0, P1), Sub(P0, P1)) / Norm(Sub(P1, P0))) - Norm(Sub(P1, P0))), odTmp > 0) {
					od = odTmp;
				}
				else if (odTmp = ((Dot(Sub(V0, P0), Sub(P1, P0)) / Norm(Sub(P1, P0))) - Norm(Sub(P1, P0))), odTmp > 0) {
					od = odTmp;
				}

				/*  */
				if (od > 0) {

					sj[a] = (Resolution / sqrt(pow((d - r[a]), 2) + pow(od, 2))) * 0.5;

				}
				else {

					sj[a] = (((Resolution) / (d - r[a]))) * 0.5;
				}
			}

		}
		//cout << "5 "<<sj[5] << endl;
		//cout << PathData.row(f) << endl;

		for (int j = 0; j < JointCount; j++) {
			result.push_back(sj[j]);
		}

	}
	return result;
}

//對兩個 4x4 齊次變換矩陣進行乘法，並將結果存入 rVal 中
void
NewtonOptimize::ArrayMul(TArray4X4 a, TArray4X4 b, TArray4X4& rVal)
{
	TArray4X4   result;

	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			float		sum = 0;

			for (int k = 0; k < 4; k++) {
				sum += (a[i][k] * b[k][j]);
			}

			result[i][j] = sum;
		}
	}
	memcpy(&rVal, &result, sizeof(rVal));
}


void
NewtonOptimize::ArrayMul(TArray4X4 a, TArray2X4 b, TArray2X4& rVal)
{
	TArray4X4   result;


	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 2; j++) {
			float		sum = 0;

			for (int k = 0; k < 4; k++) {
				sum += (a[i][k] * b[j][k]);
			}

			result[j][i] = sum;
		}
	}
	memcpy(&rVal, &result, sizeof(rVal));
}
void
NewtonOptimize::ArrayMul(TArray4X4 a, const float* bPtr, float* rVal)
{
	if ((bPtr != NULL)
		&& (rVal != NULL)) {
		TArray4     result;

		for (int i = 0; i < 4; i++) {
			float		sum = 0;
			float* aPtr = a[i];

			for (int k = 0; k < 4; k++, aPtr++) {
				sum += (*aPtr * bPtr[k]);
			}

			result[i] = sum;
		}
		memcpy(rVal, &result, sizeof(result));
	}
}
float  //計算三維向量的模長（也稱為向量的大小）。
NewtonOptimize::Norm(T3DValue a)
{
	return sqrt((a.x * a.x) + (a.y * a.y) + (a.z * a.z));
}

//計算兩個三維向量 a 和 b 的叉積，返回一個垂直於 a 和 b 的向量。
NewtonOptimize::T3DValue NewtonOptimize::Cross(T3DValue a, T3DValue b)
{
	T3DValue	c;

	c.x = (a.y * b.z) - (a.z * b.y);
	c.y = (a.z * b.x) - (a.x * b.z);
	c.z = (a.x * b.y) - (a.y * b.x);

	return c;
}
float
NewtonOptimize::Dot(T3DValue a, T3DValue b) //計算兩個三維向量 a 和 b 的點積。
{
	return (a.x * b.x) + (a.y * b.y) + (a.z * b.z);
}

//計算兩個三維向量 a 和 b 的差，返回一個新的向量。
NewtonOptimize::T3DValue NewtonOptimize::Sub(T3DValue a, T3DValue b)
{
	a.x -= b.x;
	a.y -= b.y;
	a.z -= b.z;

	return a;
}

//生成一個由起點 start 到終點 end 的平滑路徑，並根據指定的 sampleRange 進行插值。
cv::Mat NewtonOptimize::pathcreate(cv::Mat start, cv::Mat end, int sampleRange) {
	cv::Mat P1 = start;
	cv::Mat P2 = end;
	cv::Mat step;
	for (int i = 0; i < P1.cols; i++) {
		step.push_back((P1.col(i) - P2.col(i)) / sampleRange);
	}
	cv::Mat path;
	cv::Mat pos = (cv::Mat_<float>(1, 6) << 0, 0, 0, 0, 0, 0);
	path.push_back(P2);
	for (int i = 0; i < sampleRange; i++) {

		for (int j = 0; j < 6; j++) {
			pos.at<float>(0, j) = path.at<float>(i, j) + step.at<float>(j, 0);

		}
		path.push_back(pos);
	}

	return path;

}
