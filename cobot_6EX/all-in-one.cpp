//#include "NewtonUpdate.h"
//
//#include <Eigen/Dense>
//#include <iostream>
//#include <opencv2/opencv.hpp>
//#include <stdio.h>
//#include <utility>
//#include <sstream>
//#include <opencv2/core/eigen.hpp>
//#include <stdio.h>
//
//
//#include <opencv2/opencv.hpp>
//
////#include "Function.h"
//
//using namespace std;
//using namespace cv;
//using namespace ml;
//using namespace Eigen;
//
//
//NewtonOptimize::NewtonOptimize(Mat NormalizeMatrix, Mat CurrentPath_)
//{
//	cout << "Start Loading Model..." << endl;
//	MakeWorkSpace();
//	cout << "Model is Loaded!!" << endl;
//	JointRangeMatrix = NormalizeMatrix.clone();
//	deltaX = Deltax;
//	CurrentPath = CurrentPath_;
//}
//
//NewtonOptimize::~NewtonOptimize()
//{
//}
//
//void NewtonOptimize::SampleRange(float PercentageOfPath) {
//	if (PercentageOfPath >= 1) {
//		cout << "PercentageOfPath Can't Bigger Than 1" << endl;
//		return;
//	}
//	PathSampleRange = PercentageOfPath;
//}
//void NewtonOptimize::setIndex(int index_) { _index = index_; }
//int NewtonOptimize::returnIndex() { return _index; }
//
//vector<Mat> NewtonOptimize::RoutePlanning(int InputCubeX, int InputCubeY, int InputCubeZ, float SafeThreshold) {
//
//	std::clock_t begin = std::clock();
//	std::vector<std::vector<std::vector<std::vector<float>>>> AllSafeScore;
//	cubeX = InputCubeX;
//	cubeY = InputCubeY;
//	cubeZ = InputCubeZ;
//	AllSafeScore = RobotArm(CurrentPath, InputCubeX, InputCubeY, InputCubeZ);
//	JointCount = CurrentPath.cols;
//	TargetSafeThreshold = SafeThreshold;
//
//
//	//auto AllSafeScore = RobotArm(CurrentPath);
//
//	CubeSafeScore.clear();
//
//	for (size_t i = 0; i < AllSafeScore.size(); i++)
//	{
//		CubeSafeScore.push_back(AllSafeScore[i][InputCubeZ][InputCubeY][InputCubeX]);
//	}
//
//	Mat TotalSafeScore(CubeSafeScore);
//	//sprintf_s(filename, "./Result/path1Score2.csv");
//	//if (fopen_s(&outFile, filename, "a"), outFile != NULL) {
//	//	for (int i = 0; i < TotalSafeScore.rows; i++) {
//	//		fprintf(outFile, "%f\n", TotalSafeScore.at<float>(i,0));
//	//	}
//	//}
//
//	Mat SafeScoreOutOfLimit, SafeScoreOutOfLimitIndex;
//
//	threshold(TotalSafeScore, SafeScoreOutOfLimit, SafeThreshold, 1, THRESH_BINARY);
//
//	findNonZero(SafeScoreOutOfLimit, SafeScoreOutOfLimitIndex);
//
//	extractChannel(SafeScoreOutOfLimitIndex, SafeScoreOutOfLimitIndex, 1);
//
//	double MaxIndex, MinIndex;
//	minMaxIdx(SafeScoreOutOfLimitIndex, &MinIndex, &MaxIndex);
//
//	double MiddleIndex = (MinIndex + MaxIndex) * 0.5;
//	vector<int> TargetPos = { int(MinIndex) - 1,
//		//int(MinIndex + (MiddleIndex - MinIndex) * 0.25),
//							int((MinIndex + MiddleIndex) * 0.5),
//		//int(MinIndex + (MiddleIndex - MinIndex) * 0.75),
//							int(MiddleIndex),
//		//int(MiddleIndex + (MaxIndex - MiddleIndex) * 0.25),
//							int((MiddleIndex + MaxIndex) * 0.5),
//		//int(MiddleIndex + (MaxIndex - MiddleIndex) * 0.75),
//							int(MaxIndex) + 1 };
//	setIndex(MaxIndex);
//	cout << "Target Pos: ";
//	for (const auto& item : TargetPos) {
//		cout << item << ", ";
//	}
//	cout << endl;
//
//	vector<Mat> Result;
//	Mat StartPos = CurrentPath.row(TargetPos.front());
//	Result.push_back(StartPos);
//	cout << "Start Pos:" << StartPos << endl;
//
//	Mat EndPos = CurrentPath.row(TargetPos.back());
//
//	auto SafePos = OptimizePos(TargetPos);
//	for (int i = 0; i < SafePos.rows; i++) {
//		Result.push_back(SafePos.row(i));
//	}
//
//	
//	Result.push_back(EndPos);
//	cout << "End Pos:" << EndPos << endl;
//	Mat new_path;
//	for (int k = 0; k < Result.size() - 1; k++) {
//		new_path.push_back(pathcreate(Result[k + 1], Result[k], 20));
//	}
//
//	new_path.push_back(CurrentPath.rowRange(MaxIndex, CurrentPath.rows - 1));
//	cout << new_path << endl;
//	CurrentPath = new_path;
//	return Result;
//}
//Mat NewtonOptimize::Get_gk(Mat CurrentPos, Mat S) {
//	Mat gk;
//	vector<float> df = CalcDJ(CurrentPos);
//	for (size_t k = 0; k < JointCount; k++)
//	{
//		gk.push_back((float)(df[k] - 0.4 + pow(S.at<float>(0, k), 2)));
//	}
//	return gk;
//}
//
//Mat NewtonOptimize::Get_ui(Mat CurrentPos, Mat U) {
//	Mat U_pow;
//	pow(U, 2, U_pow);
//
//	Mat ui = CurrentPos - Mat::ones(CurrentPos.size(), CurrentPos.type()) + U_pow;
//
//	return ui;
//}
//
//Mat NewtonOptimize::Get_vi(Mat CurrentPos, Mat V) {
//	Mat V_pow;
//	pow(V, 2, V_pow);
//
//	Mat vi = Mat::zeros(CurrentPos.size(), CurrentPos.type()) - CurrentPos + V_pow;
//
//	return vi;
//}
//
//Mat NewtonOptimize::ExtractSubVector(Mat Data, TargetVector Target) {
//	int StartCol, EndCol;
//
//	switch (Target)
//	{
//	case NewtonOptimize::Xm:
//		StartCol = 0;
//		EndCol = JointCount;
//		break;
//	case NewtonOptimize::Xm2:
//		StartCol = JointCount;
//		EndCol = StartCol+JointCount;
//		break;
//	case NewtonOptimize::Xm3:
//		StartCol = 2 * JointCount;
//		EndCol = StartCol + JointCount;
//		break;
//	case NewtonOptimize::lota:
//		StartCol = 3*JointCount;
//		EndCol = StartCol + JointCount;
//		break;
//	case NewtonOptimize::lota2:
//		StartCol = 4*JointCount;
//		EndCol = StartCol + JointCount;
//		break;
//	case NewtonOptimize::lota3:
//		StartCol = 5*JointCount;
//		EndCol = StartCol + JointCount;
//		break;
//	case NewtonOptimize::S:
//		StartCol = 6*JointCount;
//		EndCol = StartCol + JointCount;
//		break;
//	case NewtonOptimize::S2:
//		StartCol = 7*JointCount;
//		EndCol = StartCol + JointCount;
//		break;
//	case NewtonOptimize::S3:
//		StartCol = 8*JointCount;
//		EndCol = StartCol + JointCount;
//		break;
//
//	default:
//		StartCol = 0;
//		EndCol = 0;
//		break;
//	}
//	return Data.colRange(StartCol, EndCol);
//}
//
//Mat NewtonOptimize::Normalize(Mat Src, NormalizeStatus flag) {
//	Mat Res;
//	Mat Range = JointRangeMatrix.row(1) - JointRangeMatrix.row(0);
//	if (flag == Forward) {
//		divide((Src - JointRangeMatrix.row(0)), Range, Res);
//		return Res;
//	}
//	else {
//		multiply(Src, Range, Res);
//		return Res + JointRangeMatrix.row(0);
//	}
//}
//
//Mat NewtonOptimize::DeltaNormalize(Mat Src, NormalizeStatus flag) {
//
//	Mat Res;
//	Mat Range = JointRangeMatrix.row(1) - JointRangeMatrix.row(0);
//	if (flag == Forward)
//	{
//		divide(Src, Range, Res);
//		return Res;
//	}
//	else {
//		multiply(Src, Range, Res);
//		return Res;
//	}
//}
//
//float NewtonOptimize::CalcD(size_t i, Mat Pos) {
//	//Pos = Normalize(Pos, Backward);
//	Dcount++;
//	//cout << count << endl;
//	auto Value = RobotArm(Pos, cubeX, cubeY, cubeZ);
//	return Value[0][cubeZ][cubeY][cubeX];
//}
//std::vector<float>  NewtonOptimize::CalcDJ(Mat Pos) {
//	//Pos = Normalize(Pos, Backward);
//	auto Value = RobotArmJ(Pos, cubeX, cubeY, cubeZ);
//	return Value;
//}
//Mat NewtonOptimize::OptimizePos(vector<int> TargetPos) {
//	//if (fopen_s(&outFile, filename, "w"), outFile != NULL) {
//	cout << "check" << endl;
//	Mat CurrentPos;
//
//	Mat StartPos=CurrentPath.row(TargetPos[0]);
//	Mat EndPos= CurrentPath.row(TargetPos[4]);
//	Mat Xm;
//	for (int i = 1; i < TargetPos.size()-1; i++) {
//		CurrentPos.push_back(CurrentPath.row(TargetPos[i]));
//		for (int j = 0; j < JointCount; j++) {
//			Xm.push_back(CurrentPath.at<float>(TargetPos[i], j));
//		}
//		
//	}
//	Mat l1 = Mat::ones(1, JointCount, CV_32F);
//	Mat S1 = Mat::ones(1, JointCount, CV_32F);
//	Mat l2 = Mat::ones(1, JointCount, CV_32F);
//	Mat S2 = Mat::ones(1, JointCount, CV_32F);
//	Mat l3 = Mat::ones(1, JointCount, CV_32F);
//	Mat S3 = Mat::ones(1, JointCount, CV_32F);
//
//
//	l1 = l1 * lagragian_l;
//	S1 = S1 * slack_S;
//	l2 = l2 * lagragian_l;
//	S2 = S2 * slack_S;
//	l3 = l3 * lagragian_l;
//	S3 = S3 * slack_S;
//
//	Xm.push_back(l1.t());
//	Xm.push_back(l2.t());
//	Xm.push_back(l3.t());
//	Xm.push_back(S1.t());
//	Xm.push_back(S2.t());
//	Xm.push_back(S3.t());
//
//	Mat PreviousIterPos = Xm.clone();
//
//	float ResultSafetyScore = 0;
//	for (size_t i = 1; i < MaxIterLimit; i++)
//	{
//		float ResultSafetyScoretmp1 = 0;
//		float ResultSafetyScoretmp2 = 0;
//		float ResultSafetyScoretmp3 = 0;
//		Mat GradientMatrix = CalcGradient(Xm, CurrentPos, StartPos, EndPos);
//		Mat HessianMatrix = CalcHessian(Xm, CurrentPos.row(0), CurrentPos.row(1), CurrentPos.row(2), StartPos, EndPos);
//		//cout << GradientMatrix.at<float>(1,0) <<endl;
//		//Dcount = 0;
//		Mat h = HessianMatrix.inv();
//		Mat NewIterPos = Xm - HessianMatrix.inv() * GradientMatrix;
//
//		//for (size_t i = 0; i < CubeCount; i++)//Pos Count
//		//{
//		//	ResultSafetyScoretmp += CalcD(i, ExtractSubVector(NewIterPos.t(), TargetVector::Xm));
//		//}
//		//ResultSafetyScoretmp = ResultSafetyScoretmp / (float)CubeCount;
//
//		auto NewPosXm1 = ExtractSubVector(NewIterPos.t(), TargetVector::Xm);
//		auto NewPosXm2 = ExtractSubVector(NewIterPos.t(), TargetVector::Xm2);
//		auto NewPosXm3 = ExtractSubVector(NewIterPos.t(), TargetVector::Xm3);
//
//		ResultSafetyScoretmp1 = CalcD(i, NewPosXm1);
//		ResultSafetyScoretmp2 = CalcD(i, NewPosXm2);
//		ResultSafetyScoretmp3 = CalcD(i, NewPosXm3);
//		
//		float totalScore = ResultSafetyScoretmp1 + ResultSafetyScoretmp2 + ResultSafetyScoretmp3;
//		float avg = totalScore / 3;
//		//ResultSafetyScoretmp = ResultSafetyScoretmp / (float)CubeCount;
//		//auto NewPosXm = Normalize(ExtractSubVector(NewIterPos.t(), TargetVector::Xm), Backward);
//		//for (int i = 0; i < NewPosXm.cols; i++) {
//		//	fprintf(outFile, "%f,", NewPosXm.at<float>(0, i));
//		//}
//		//fprintf(outFile, "%f\n", ResultSafetyScoretmp);
//		cout << NewPosXm1 << endl;
//		cout << NewPosXm2 << endl;
//		cout << NewPosXm3 << endl;
//		cout << ResultSafetyScoretmp1 <<" "<< ResultSafetyScoretmp2 <<" "<< ResultSafetyScoretmp3 << endl;
//		//if (false) {
//		if (avg < 0.40f && abs(avg - ResultSafetyScore) < 0.001) {
//			//cout << "Final Pos: " << endl;
//			//cout << ResultSafetyScoretmp << endl;
//			//cout << CurrentPos << endl;
//			//cout << NewPosXm << endl;
//			//cout << i << endl;
//			Mat NewPosXm;
//			NewPosXm.push_back(NewPosXm1);
//			NewPosXm.push_back(NewPosXm2);
//			NewPosXm.push_back(NewPosXm3);
//			avg = 0;
//			ResultSafetyScore = 0;
//			return NewPosXm;
//
//		}
//
//		ResultSafetyScore = avg;
//		PreviousIterPos = Xm.clone();
//		Xm = NewIterPos.clone();
//	}
//	//}
//	cout << "shit" << endl;
//	return Mat();
//}
//
//float NewtonOptimize::FunctionXm(Mat CurrentPos1, Mat OriPos1, Mat CurrentPos2, Mat OriPos2, Mat CurrentPos3, Mat OriPos3, Mat StartPos, Mat EndPos) {
//	Mat PowBetween1, PowBetween2, PowCurrentStart, PowCurrentEnd, PowPos1, PowPos2, PowPos3;
//	pow(norm(CurrentPos1, OriPos1), 2, PowPos1);
//	pow(norm(CurrentPos2, OriPos2), 2, PowPos2);
//	pow(norm(CurrentPos3, OriPos3), 2, PowPos3);
//	pow(norm(CurrentPos1, StartPos), 2, PowCurrentStart);
//	pow(norm(CurrentPos3, EndPos), 2, PowCurrentEnd);
//	pow(norm(CurrentPos1, CurrentPos2), 2, PowBetween1);
//	pow(norm(CurrentPos2, CurrentPos3), 2, PowBetween2);
//
//	return sum(PowPos1 + PowPos2 + PowPos3 + h_weight * PowCurrentStart + t_weight * PowCurrentEnd + w1 * PowBetween1 + w2 * PowBetween2)[0];
//}
////float NewtonOptimize::FunctionXm(Mat CurrentPos1, Mat OriPos1, Mat CurrentPos2, Mat OriPos2, Mat CurrentPos3, Mat OriPos3, Mat StartPos, Mat EndPos) {
////	float PowBetween1, PowBetween2, PowCurrentStart, PowCurrentEnd, PowPos1, PowPos2, PowPos3;
////	PowPos1 = norm(CurrentPos1, OriPos1);
////	PowPos2 = norm(CurrentPos2, OriPos2);
////	PowPos3 = norm(CurrentPos3, OriPos3);
////	PowCurrentStart = norm(CurrentPos1, StartPos);
////	PowCurrentEnd = norm(CurrentPos3, EndPos);
////	PowBetween1 = norm(CurrentPos1, CurrentPos2);
////	PowBetween2 = norm(CurrentPos2, CurrentPos3);
////
////	return PowPos1 + PowPos2 + PowPos3 + h_weight * PowCurrentStart + t_weight * PowCurrentEnd + w1 * PowBetween1 + w2 * PowBetween2;
////}
//float NewtonOptimize::FunctionXm(Mat CurrentPos, Mat OriPos, Mat StartPos, Mat EndPos) {
//	Mat PowCurrentOri, PowCurrentStart, PowCurrentEnd, OLDMPFC, NEWMPFC, CenterPoint;
//	pow(norm(CurrentPos, OriPos), 2, PowCurrentOri);
//	pow(norm(CurrentPos, StartPos), 2, PowCurrentStart);
//	pow(norm(CurrentPos, EndPos), 2, PowCurrentEnd);
//	//PowCurrentOri = norm(CurrentPos, OriPos);
//	//PowCurrentStart = norm(CurrentPos, StartPos);
//	//PowCurrentEnd = norm(CurrentPos, EndPos);
//	//CenterPoint = (Mat_<float>(1, 6) << -8.0002, -46.662866, -25.617466, 0, -25.518116, 26.896169);
//	//CenterPoint = (Mat_<float>(1, 6) << 5.000, -38.860382, 18.78968, 0, -11.862453, -0.000113);
//	//pow(norm(CurrentPos, CenterPoint), 1, OLDMPFC);
//	//pow(norm(OriPos, CenterPoint), 1, NEWMPFC);
//	//double NewCurrentMPFCUP = (CurrentPos - CenterPoint).dot((OriPos - CenterPoint));
//	//double NewCurrentMPFCDN = OLDMPFC.dot(NEWMPFC);
//	//double NewCurrentMPFC = NewCurrentMPFCUP / NewCurrentMPFCDN;
//
//	//return  sum(f_weight * PowCurrentOri + h_weight * PowCurrentStart + t_weight * PowCurrentEnd + new_weight * NewCurrentMPFC)[0];
//	return  sum(f_weight * PowCurrentOri + h_weight * PowCurrentStart + t_weight * PowCurrentEnd)[0];
//	//return  f_weight * PowCurrentOri + h_weight * PowCurrentStart + t_weight * PowCurrentEnd;
//
//}
////here  /////here  /////here  /////here  /////here  /////here  /////here  /////here  /////here  /////here  /////here  ///
//Mat NewtonOptimize::PosDelta(Mat CurrentPos, int i, int j) {
//	Mat Out = CurrentPos.clone();
//	Out.at<float>(0, i) += deltaX.at<float>(0, i);
//	Out.at<float>(0, j) += deltaX.at<float>(0, j);
//	return Out;
//}
//
//Mat NewtonOptimize::PosDelta(Mat CurrentPos, int i, Direction Dir) {
//	Mat Out = CurrentPos.clone();
//	if (Dir == Plus) {
//		Out.at<float>(0, i) += deltaX.at<float>(0, i);
//	}
//	else {
//		Out.at<float>(0, i) -= deltaX.at<float>(0, i);
//	}
//	return Out;
//}
////Mat NewtonOptimize::CalcGradient(Mat CurrentPos, Mat OriPos1, Mat OriPos2, Mat OriPos3, Mat StartPos, Mat EndPos) {
////	CurrentPos = CurrentPos.t();
////
////
////	Mat Gradient;
////	Mat XmVector1 = ExtractSubVector(CurrentPos, Xm);
////	Mat SVector1 = ExtractSubVector(CurrentPos, S);
////	Mat UVector1 = ExtractSubVector(CurrentPos, U);
////	Mat VVector1 = ExtractSubVector(CurrentPos, V);
////	Mat lotaVector1 = ExtractSubVector(CurrentPos, lota);
////	Mat muVector1 = ExtractSubVector(CurrentPos, mu);
////	Mat tauVector1 = ExtractSubVector(CurrentPos, tau);
////
////}
//
//Mat NewtonOptimize::CalcGradient(Mat CurrentPos, Mat OriPos, Mat StartPos, Mat EndPos) {
//	CurrentPos = CurrentPos.t();
//
//	Mat Gradient;
//	Mat XmVector1 = ExtractSubVector(CurrentPos, Xm);
//	Mat XmVector2 = ExtractSubVector(CurrentPos, Xm2);
//	Mat XmVector3 = ExtractSubVector(CurrentPos, Xm3);
//	Mat SVector1 = ExtractSubVector(CurrentPos, S);
//	Mat SVector2 = ExtractSubVector(CurrentPos, S2);
//	Mat SVector3 = ExtractSubVector(CurrentPos, S3);
//	Mat lotaVector1 = ExtractSubVector(CurrentPos, lota);
//	Mat lotaVector2 = ExtractSubVector(CurrentPos, lota2);
//	Mat lotaVector3 = ExtractSubVector(CurrentPos, lota3);
//
//	int N = JointCount;
//	//X1
//	Mat DeltaFXm1 = Mat::zeros(1, N, CV_32F);
//	for (size_t i = 0; i < N; i++)
//	{
//		float gfm = FunctionXm(XmVector1, OriPos.row(0), XmVector2, OriPos.row(1), XmVector3, OriPos.row(2), StartPos, EndPos);
//		float gfm_p_ei = FunctionXm(PosDelta(XmVector1, i, Plus), OriPos.row(0), XmVector2, OriPos.row(1), XmVector3, OriPos.row(2), StartPos, EndPos);
//		DeltaFXm1.at<float>(0, i) = ((gfm_p_ei - gfm) / deltaX.at<float>(0, i));
//	}
//
//	Mat LotaDeltaFXmD1;
//
//	for (size_t k = 0; k < JointCount; k++)//PosCount
//	{
//		float DeltaDXmi = 0;
//		auto Dxm = CalcDJ(XmVector1);
//		auto PosDxm = CalcDJ(PosDelta(XmVector1, k, Plus));
//		for (size_t i = 0; i < JointCount; i++)
//		{
//			DeltaDXmi += lotaVector1.at<float>(0, i) * (PosDxm[i] - Dxm[i]) / deltaX.at<float>(0, i);
//		}
//		LotaDeltaFXmD1.push_back(DeltaDXmi);
//	}
//	//X2
//	Mat DeltaFXm2 = Mat::zeros(1, N, CV_32F);
//	for (size_t i = 0; i < N; i++)
//	{
//		float gfm = FunctionXm(XmVector1, OriPos.row(0), XmVector2, OriPos.row(1), XmVector3,  OriPos.row(2), StartPos, EndPos);
//		float gfm_p_ei = FunctionXm(XmVector1, OriPos.row(0), PosDelta(XmVector2, i, Plus), OriPos.row(1), XmVector3,  OriPos.row(2), StartPos, EndPos);
//		DeltaFXm2.at<float>(0, i) = ((gfm_p_ei - gfm) / deltaX.at<float>(0, i));
//	}
//
//	Mat LotaDeltaFXmD2;
//
//	for (size_t k = 0; k < JointCount; k++)//PosCount
//	{
//		float DeltaDXmi = 0;
//		auto Dxm = CalcDJ(XmVector2);
//		auto PosDxm = CalcDJ(PosDelta(XmVector2, k, Plus));
//		for (size_t i = 0; i < JointCount; i++)
//		{
//			DeltaDXmi += lotaVector2.at<float>(0, i) * (PosDxm[i] - Dxm[i]) / deltaX.at<float>(0, i);
//		}
//		LotaDeltaFXmD2.push_back(DeltaDXmi);
//	}
//	//X3
//	Mat DeltaFXm3 = Mat::zeros(1, N, CV_32F);
//	for (size_t i = 0; i < N; i++)
//	{
//		float gfm = FunctionXm(XmVector1, OriPos.row(0), XmVector2, OriPos.row(1), XmVector3,OriPos.row(2), StartPos, EndPos);
//		float gfm_p_ei = FunctionXm(XmVector1, OriPos.row(0), XmVector2, OriPos.row(1), PosDelta(XmVector3, i, Plus),OriPos.row(2), StartPos, EndPos);
//		DeltaFXm3.at<float>(0, i) = ((gfm_p_ei - gfm) / deltaX.at<float>(0, i));
//	}
//
//	Mat LotaDeltaFXmD3;
//
//	for (size_t k = 0; k < JointCount; k++)//PosCount
//	{
//		float DeltaDXmi = 0;
//		auto Dxm = CalcDJ(XmVector3);
//		auto PosDxm = CalcDJ(PosDelta(XmVector3, k, Plus));
//		for (size_t i = 0; i < JointCount; i++)
//		{
//			DeltaDXmi += lotaVector3.at<float>(0, i) * (PosDxm[i] - Dxm[i]) / deltaX.at<float>(0, i);
//		}
//		LotaDeltaFXmD3.push_back(DeltaDXmi);
//	}
//
//	Mat Gradient_Xm1 = DeltaFXm1 + LotaDeltaFXmD1.t(); /*+ muVector - tauVector*/;
//	Mat Gradient_Xm2 = DeltaFXm2 + LotaDeltaFXmD2.t();
//	Mat Gradient_Xm3 = DeltaFXm3 + LotaDeltaFXmD3.t();
//	Mat Gradient_L1 = Get_gk(XmVector1, SVector1);
//	Mat Gradient_L2 = Get_gk(XmVector2, SVector2);
//	Mat Gradient_L3 = Get_gk(XmVector3, SVector3);
//	Mat Gradient_S1 = 2 * lotaVector1.mul(SVector1);
//	Mat Gradient_S2 = 2 * lotaVector2.mul(SVector2);
//	Mat Gradient_S3 = 2 * lotaVector3.mul(SVector3);
//
//	Gradient.push_back(Gradient_Xm1.t());
//	Gradient.push_back(Gradient_Xm2.t());
//	Gradient.push_back(Gradient_Xm3.t());
//	Gradient.push_back(Gradient_L1);
//	Gradient.push_back(Gradient_L2);
//	Gradient.push_back(Gradient_L3);
//	Gradient.push_back(Gradient_S1.t());
//	Gradient.push_back(Gradient_S2.t());
//	Gradient.push_back(Gradient_S3.t());
//	return Gradient;
//}
//Mat NewtonOptimize::CalcHessian(cv::Mat CurrentPos, cv::Mat OriPos1, cv::Mat OriPos2, cv::Mat OriPos3, cv::Mat StartPos, cv::Mat EndPos){
//	int P = CubeCount;
//	int N = JointCount;
//	CurrentPos = CurrentPos.t();
//	Mat XmVector1 = ExtractSubVector(CurrentPos, Xm);
//	Mat XmVector2 = ExtractSubVector(CurrentPos, Xm2);
//	Mat XmVector3 = ExtractSubVector(CurrentPos, Xm3);
//	Mat SVector1 = ExtractSubVector(CurrentPos, S);
//	Mat SVector2 = ExtractSubVector(CurrentPos, S2);
//	Mat SVector3 = ExtractSubVector(CurrentPos, S3);
//	Mat lotaVector1 = ExtractSubVector(CurrentPos, lota);
//	Mat lotaVector2 = ExtractSubVector(CurrentPos, lota2);
//	Mat lotaVector3 = ExtractSubVector(CurrentPos, lota3);
//
//	Mat Mu_Tau = Mat::zeros(N, N, CV_32F);
//	Mat L_Xm1 = Mat::zeros(N, N, CV_32F);
//	for (size_t i = 0; i < N; i++)
//	{
//		for (size_t j = 0; j < N; j++)
//		{
//			float fm = FunctionXm(XmVector1, OriPos1, XmVector2, OriPos2, XmVector3, OriPos3, StartPos, EndPos);
//			float fm_p_ei = FunctionXm(PosDelta(XmVector1, i, Plus), OriPos1, XmVector2, OriPos2, XmVector3, OriPos3, StartPos, EndPos);
//			float fm_m_ei = FunctionXm(PosDelta(XmVector1, i, Minus), OriPos1, XmVector2, OriPos2, XmVector3, OriPos3, StartPos, EndPos);
//			float fm_p_ej = FunctionXm(PosDelta(XmVector1, j, Plus), OriPos1, XmVector2, OriPos2, XmVector3, OriPos3, StartPos, EndPos);
//			float fm_p_ei_ej = FunctionXm(PosDelta(XmVector1, i, j), OriPos1, XmVector2, OriPos2, XmVector3, OriPos3, StartPos, EndPos);
//			float Dm_Xmi_Xmj = 0;
//			if (i == j) {  // i == j
//				vector<float> Dm = CalcDJ(XmVector1);
//				vector<float> Dm_p_ei = CalcDJ(PosDelta(XmVector1, i, Plus));
//				vector<float> Dm_m_ei = CalcDJ(PosDelta(XmVector1, i, Minus));
//				for (size_t k = 0; k < N; k++)
//				{
//					Dm_Xmi_Xmj += lotaVector1.at<float>(0, k) * ((Dm_p_ei[k] - 2 * Dm[k] + Dm_m_ei[k]) / pow(Delta, 2));
//				}
//				L_Xm1.at<float>(j, i) = ((fm_p_ei - 2 * fm + fm_m_ei) / pow(Delta, 2)) + Dm_Xmi_Xmj;
//			}
//			else {
//				vector<float> Dm = CalcDJ(XmVector1);
//				vector<float> Dm_p_ei = CalcDJ(PosDelta(XmVector1, i, Plus));
//				vector<float> Dm_p_ej = CalcDJ(PosDelta(XmVector1, j, Plus));
//				vector<float> Dm_p_ei_ej = CalcDJ(PosDelta(XmVector1, i, j));
//				for (size_t k = 0; k < N; k++)
//				{
//					Dm_Xmi_Xmj += lotaVector1.at<float>(0, k) * ((Dm_p_ei_ej[k] - Dm_p_ei[k] - Dm_p_ej[k] + Dm[k]) / pow(Delta, 2));
//				}
//				L_Xm1.at<float>(j, i) = (fm_p_ei_ej - fm_p_ei - fm_p_ej + fm) / pow(Delta, 2) + Dm_Xmi_Xmj;
//			}
//		}
//	}
//	//X2
//	Mat L_Xm2 = Mat::zeros(N, N, CV_32F);
//	for (size_t i = 0; i < N; i++)
//	{
//		for (size_t j = 0; j < N; j++)
//		{
//			float fm = FunctionXm(XmVector1, OriPos1, XmVector2, OriPos2, XmVector3, OriPos3, StartPos, EndPos);
//			float fm_p_ei = FunctionXm(XmVector1, OriPos1, PosDelta(XmVector2, i, Plus), OriPos2, XmVector3, OriPos3, StartPos, EndPos);
//			float fm_m_ei = FunctionXm(XmVector1, OriPos1, PosDelta(XmVector2, i, Minus), OriPos2, XmVector3, OriPos3, StartPos, EndPos);
//			float fm_p_ej = FunctionXm(XmVector1, OriPos1, PosDelta(XmVector2, j, Plus), OriPos2, XmVector3, OriPos3, StartPos, EndPos);
//			float fm_p_ei_ej = FunctionXm(XmVector1, OriPos1, PosDelta(XmVector2, i, j), OriPos2, XmVector3, OriPos3, StartPos, EndPos);
//			float Dm_Xmi_Xmj = 0;
//			if (i == j) {  // i == j
//				vector<float> Dm = CalcDJ(XmVector2);
//				vector<float> Dm_p_ei = CalcDJ(PosDelta(XmVector2, i, Plus));
//				vector<float> Dm_m_ei = CalcDJ(PosDelta(XmVector2, i, Minus));
//				for (size_t k = 0; k < N; k++)
//				{
//					Dm_Xmi_Xmj += lotaVector2.at<float>(0, k) * ((Dm_p_ei[k] - 2 * Dm[k] + Dm_m_ei[k]) / pow(Delta, 2));
//				}
//				L_Xm2.at<float>(j, i) = ((fm_p_ei - 2 * fm + fm_m_ei) / pow(Delta, 2)) + Dm_Xmi_Xmj;
//			}
//			else {
//				vector<float> Dm = CalcDJ(XmVector2);
//				vector<float> Dm_p_ei = CalcDJ(PosDelta(XmVector2, i, Plus));
//				vector<float> Dm_p_ej = CalcDJ(PosDelta(XmVector2, j, Plus));
//				vector<float> Dm_p_ei_ej = CalcDJ(PosDelta(XmVector2, i, j));
//				for (size_t k = 0; k < N; k++)
//				{
//					Dm_Xmi_Xmj += lotaVector2.at<float>(0, k) * ((Dm_p_ei_ej[k] - Dm_p_ei[k] - Dm_p_ej[k] + Dm[k]) / pow(Delta, 2));
//				}
//				L_Xm2.at<float>(j, i) = (fm_p_ei_ej - fm_p_ei - fm_p_ej + fm) / pow(Delta, 2) + Dm_Xmi_Xmj;
//			}
//		}
//	}
//	//X3
//	Mat L_Xm3 = Mat::zeros(N, N, CV_32F);
//	for (size_t i = 0; i < N; i++)
//	{
//		for (size_t j = 0; j < N; j++)
//		{
//			float fm = FunctionXm(XmVector1, OriPos1, XmVector2, OriPos2, XmVector3, OriPos3, StartPos, EndPos);
//			float fm_p_ei = FunctionXm(XmVector1, OriPos1, XmVector2, OriPos2, PosDelta(XmVector3, i, Plus), OriPos3, StartPos, EndPos);
//			float fm_m_ei = FunctionXm(XmVector1, OriPos1, XmVector2, OriPos2, PosDelta(XmVector3, i, Minus), OriPos3, StartPos, EndPos);
//			float fm_p_ej = FunctionXm(XmVector1, OriPos1, XmVector2, OriPos2, PosDelta(XmVector3, j, Plus), OriPos3, StartPos, EndPos);
//			float fm_p_ei_ej = FunctionXm(XmVector1, OriPos1, XmVector2, OriPos2, PosDelta(XmVector3, i, j), OriPos3, StartPos, EndPos);
//			float Dm_Xmi_Xmj = 0;
//			if (i == j) {  // i == j
//				vector<float> Dm = CalcDJ(XmVector3);
//				vector<float> Dm_p_ei = CalcDJ(PosDelta(XmVector3, i, Plus));
//				vector<float> Dm_m_ei = CalcDJ(PosDelta(XmVector3, i, Minus));
//				for (size_t k = 0; k < N; k++)
//				{
//					Dm_Xmi_Xmj += lotaVector3.at<float>(0, k) * ((Dm_p_ei[k] - 2 * Dm[k] + Dm_m_ei[k]) / pow(Delta, 2));
//				}
//				L_Xm3.at<float>(j, i) = ((fm_p_ei - 2 * fm + fm_m_ei) / pow(Delta, 2)) + Dm_Xmi_Xmj;
//			}
//			else {
//				vector<float> Dm = CalcDJ(XmVector3);
//				vector<float> Dm_p_ei = CalcDJ(PosDelta(XmVector3, i, Plus));
//				vector<float> Dm_p_ej = CalcDJ(PosDelta(XmVector3, j, Plus));
//				vector<float> Dm_p_ei_ej = CalcDJ(PosDelta(XmVector3, i, j));
//				for (size_t k = 0; k < N; k++)
//				{
//					Dm_Xmi_Xmj += lotaVector3.at<float>(0, k) * ((Dm_p_ei_ej[k] - Dm_p_ei[k] - Dm_p_ej[k] + Dm[k]) / pow(Delta, 2));
//				}
//				L_Xm3.at<float>(j, i) = (fm_p_ei_ej - fm_p_ei - fm_p_ej + fm) / pow(Delta, 2) + Dm_Xmi_Xmj;
//			}
//		}
//	}
//	Mat X1_X2 = Mat::zeros(N,N,CV_32F);
//	for (int i = 0; i < JointCount; i++){
//		float fm = FunctionXm(XmVector1, OriPos1, XmVector2, OriPos2, XmVector3, OriPos3, StartPos, EndPos);
//		float fm_p_ei = FunctionXm(PosDelta(XmVector1, i, Plus), OriPos1, XmVector2, OriPos2, XmVector3, OriPos3, StartPos, EndPos);
//		for(int j=0 ;j<JointCount;j++){
//			float fm_p_ej = FunctionXm(XmVector1, OriPos1, PosDelta(XmVector2, j, Plus), OriPos2, XmVector3, OriPos3, StartPos, EndPos);
//			float fm_p_ei_ej = FunctionXm(PosDelta(XmVector1, i, Plus), OriPos1, PosDelta(XmVector2, j, Plus), OriPos2, XmVector3, OriPos3, StartPos, EndPos);
//			X1_X2.at<float>(i,j) = (fm_p_ei_ej - fm_p_ei - fm_p_ej + fm) / pow(Delta, 2);
//		}
//	}
//
//	Mat X2_X3 = Mat::zeros(N, N, CV_32F);
//	for (int i = 0; i < JointCount; i++) {
//		float fm = FunctionXm(XmVector1, OriPos1, XmVector2, OriPos2, XmVector3, OriPos3, StartPos, EndPos);
//		float fm_p_ei = FunctionXm(XmVector1, OriPos1, PosDelta(XmVector2, i, Plus), OriPos2, XmVector3, OriPos3, StartPos, EndPos);
//		for (int j = 0; j < JointCount; j++) {
//			float fm_p_ej = FunctionXm(XmVector1, OriPos1, XmVector2, OriPos2, PosDelta(XmVector3, j, Plus), OriPos3, StartPos, EndPos);
//			float fm_p_ei_ej = FunctionXm(XmVector1, OriPos1, PosDelta(XmVector2, i, Plus), OriPos2, PosDelta(XmVector3, j, Plus), OriPos3, StartPos, EndPos);
//			X2_X3.at<float>(i, j) = (fm_p_ei_ej - fm_p_ei - fm_p_ej + fm) / pow(Delta, 2);
//		}
//	}
//	//x1
//	Mat L_Xm_lota1 = Mat::zeros(N, N, CV_32F);
//	for (size_t k = 0; k < N; k++)
//	{
//		vector<float> Dm = CalcDJ(XmVector1);
//		vector<float> Dm_p_ei = CalcDJ(PosDelta(XmVector1, k, Plus));
//		for (size_t i = 0; i < N; i++)
//		{
//			L_Xm_lota1.at<float>(k, i) = (Dm_p_ei[i] - Dm[i]) / Delta;//���l54
//		}
//	}
//	//X2
//	Mat L_Xm_lota2 = Mat::zeros(N, N, CV_32F);
//	for (size_t k = 0; k < N; k++)
//	{
//		vector<float> Dm = CalcDJ(XmVector2);
//		vector<float> Dm_p_ei = CalcDJ(PosDelta(XmVector2, k, Plus));
//		for (size_t i = 0; i < N; i++)
//		{
//			L_Xm_lota2.at<float>(k, i) = (Dm_p_ei[i] - Dm[i]) / Delta;//���l54
//		}
//	}
//	//X3
//	Mat L_Xm_lota3 = Mat::zeros(N, N, CV_32F);
//	for (size_t k = 0; k < N; k++)
//	{
//		vector<float> Dm = CalcDJ(XmVector3);
//		vector<float> Dm_p_ei = CalcDJ(PosDelta(XmVector3, k, Plus));
//		for (size_t i = 0; i < N; i++)
//		{
//			L_Xm_lota3.at<float>(k, i) = (Dm_p_ei[i] - Dm[i]) / Delta;//���l54
//		}
//	}
//
//	Mat L_lota_S1 = Mat::zeros(N, N, CV_32F);
//	for (size_t k = 0; k < N; k++)
//	{
//		L_lota_S1.at<float>(k, k) = 2 * SVector1.at<float>(0, k);
//	}
//	Mat L_lota_S2 = Mat::zeros(N, N, CV_32F);
//	for (size_t k = 0; k < N; k++)
//	{
//		L_lota_S2.at<float>(k, k) = 2 * SVector2.at<float>(0, k);
//	}
//	Mat L_lota_S3 = Mat::zeros(N, N, CV_32F);
//	for (size_t k = 0; k < N; k++)
//	{
//		L_lota_S3.at<float>(k, k) = 2 * SVector3.at<float>(0, k);
//	}
//	Mat L_S1 = Mat::zeros(N, N, CV_32F);
//	for (size_t k = 0; k < N; k++)
//	{
//		L_S1.at<float>(k, k) = 2 * lotaVector1.at<float>(0, k);
//	}
//	Mat L_S2 = Mat::zeros(N, N, CV_32F);
//	for (size_t k = 0; k < N; k++)
//	{
//		L_S2.at<float>(k, k) = 2 * lotaVector2.at<float>(0, k);
//	}
//	Mat L_S3 = Mat::zeros(N, N, CV_32F);
//	for (size_t k = 0; k < N; k++)
//	{
//		L_S3.at<float>(k, k) = 2 * lotaVector3.at<float>(0, k);
//	}
//
//	Mat H = Mat::zeros(9 * N, 9 * N, CV_32F);
//	L_Xm1.copyTo(H(Rect(Point(0, 0), L_Xm1.size())));
//	X1_X2.copyTo(H(Rect(Point(N, 0), X1_X2.size())));
//	L_Xm2.copyTo(H(Rect(Point(N, N), L_Xm2.size())));
//	X2_X3.copyTo(H(Rect(Point(2*N, N), X2_X3.size())));
//	L_Xm3.copyTo(H(Rect(Point(2*N, 2*N), L_Xm3.size())));
//	L_Xm_lota1.copyTo(H(Rect(Point(3*N, 0), L_Xm_lota1.size())));
//	L_Xm_lota2.copyTo(H(Rect(Point(4*N, N), L_Xm_lota2.size())));
//	L_Xm_lota3.copyTo(H(Rect(Point(5*N, 2*N), L_Xm_lota3.size())));
//	L_lota_S1.copyTo(H(Rect(Point(6*N, 3*N), L_lota_S1.size())));
//	L_lota_S2.copyTo(H(Rect(Point(7 * N, 4 * N), L_lota_S1.size())));
//	L_lota_S3.copyTo(H(Rect(Point(8 * N, 5 * N), L_lota_S1.size())));
//	L_S1.copyTo(H(Rect(Point(6* N, 6*N), L_S1.size())));
//	L_S2.copyTo(H(Rect(Point(7 * N, 7 * N), L_S1.size())));
//	L_S3.copyTo(H(Rect(Point(8 * N, 8 * N), L_S1.size())));
//	//L_mu_U.copyTo(H(Rect(Point(2 * N + 2 * P, N + 2 * P), L_mu_U.size())));
//	//L_U.copyTo(H(Rect(Point(2 * N + 2 * P, 2 * N + 2 * P), L_U.size())));
//	//L_tau_V.copyTo(H(Rect(Point(4 * N + 2 * P, 3 * N + 2 * P), L_tau_V.size())));
//	//L_V.copyTo(H(Rect(Point(4 * N + 2 * P, 4 * N + 2 * P), L_V.size())));
//	completeSymm(H);
//	return H;
//}
//
//void NewtonOptimize::MakeWorkSpace(void) {
//	for (long z = 0; z < LWH_SIZE_Z; z++) {
//		for (long y = 0; y < LWH_SIZE_XY; y++) {
//			for (long x = 0; x < LWH_SIZE_XY; x++) {
//				WS[z][y][x].x = (LWH_XY_MIN + (x * RESOLUTION));
//				WS[z][y][x].y = (LWH_XY_MIN + (y * RESOLUTION));
//				WS[z][y][x].z = (LWH_Z_MIN + (z * RESOLUTION));
//			}
//		}
//	}
//	for (long z = 0; z < LWH_SIZE_Z - 1; z++) {
//		for (long y = 0; y < LWH_SIZE_XY - 1; y++) {
//			for (long x = 0; x < LWH_SIZE_XY - 1; x++) {
//				for (int k = 0; k < 8; k++) {
//					box[z][y][x][0] = WS[z][y][x];
//					box[z][y][x][1] = WS[z][y][x + 1];
//					box[z][y][x][2] = WS[z][y + 1][x + 1];
//					box[z][y][x][3] = WS[z][y + 1][x];
//					box[z][y][x][4] = WS[z + 1][y][x];
//					box[z][y][x][5] = WS[z + 1][y][x + 1];
//					box[z][y][x][6] = WS[z + 1][y + 1][x + 1];
//					box[z][y][x][7] = WS[z + 1][y + 1][x];
//
//					/* dectectPt(i,j,k,1,:)=[(WS(i,j,k,:)+WS(i+1,j+1,k+1,:))./2];%�����I */
//					dectectPt[z][y][x].x = ((float)(WS[z][y][x].x + WS[z + 1][y + 1][x + 1].x)) / 2;
//					dectectPt[z][y][x].y = ((float)(WS[z][y][x].y + WS[z + 1][y + 1][x + 1].y)) / 2;
//					dectectPt[z][y][x].z = ((float)(WS[z][y][x].z + WS[z + 1][y + 1][x + 1].z)) / 2;
//				}
//			}
//		}
//	}
//}
//
//std::vector<std::vector<std::vector<std::vector<float>>>> NewtonOptimize::RobotArm(cv::Mat PathData, int cubeX, int cubeY, int cubeZ)
//{
//	static const float	r[] = { 112.935,107.26,126.5,84.63,71.77,52.74, };
//	float	    Resolution = sqrt(((RESOLUTION / 2) * (RESOLUTION / 2)) * 3);
//	TArray4X4	tmp, tmpC;
//	TArray2X4	link[7];
//
//	float		sj[CountOfArray(link)];
//	bool		sj01[CountOfArray(sj)];
//	float       C[6];
//	vector<string> row;
//	//string line, word;
//	auto output = std::vector<std::vector<std::vector<std::vector<float>>>>(PathData.rows, std::vector<std::vector<std::vector<float>>>(CountOfArray(box), std::vector<std::vector<float>>(CountOfArray(box[0]), std::vector<float>(CountOfArray(box[0][0])))));
//
//	for (int f = 0; f < PathData.rows; f++)
//	{
//		for (int j = 0; j < PathData.row(f).cols; j++)
//		{
//			C[j] = PathData.at<float>(f, j);
//		}
//		float	J[] =
//		{
//			float(((C[0]) / 180.0) * M_PI),
//			float(((C[1]) / 180.0) * M_PI),
//			float(((C[2]) / 180.0) * M_PI),
//			float(((C[3]) / 180.0) * M_PI),
//			float(((C[4]) / 180.0) * M_PI),
//			float(((C[5]) / 180.0) * M_PI),
//		};
//
//		TArray2X4	link0 =
//		{
//			{ 0, 0, 0, 1, },
//			{ 0, 0, 190, 1, },
//		};
//
//		// L1
//		TArray2X4	link1 =
//		{
//			{ 0, 0, 0, 1, },
//			{ 0, 71.72, 242.41, 1, },
//		};
//
//		TArray4X4	C1 =
//		{
//			{ cos(J[0]), -sin(J[0]), 0, 0, },
//			{ sin(J[0]), cos(J[0]), 0, 0, },
//			{ 0, 0, 1, 0, },
//			{ 0, 0, 0, 1, },
//		};
//
//		TArray4X4	T10 =
//		{
//			{ 1, 0, 0, 0, },
//			{ 0, 1, 0, 0, },
//			{ 0, 0, 1, 190,},
//			{ 0, 0, 0, 1, },
//		};
//
//		// L2
//		TArray2X4		link2 =
//		{
//			{ 0, 0.02, -76.55, 1, },
//			{ 0, -0.15, 480.47, 1, },
//		};
//
//		TArray4X4		C2 =
//		{
//			{1, 0, 0, 0, },
//			{ 0, cos(J[1]), -sin(J[1]), 0, },
//			{ 0, sin(J[1]), cos(J[1]), 0, },
//			{ 0, 0, 0, 1, },
//		};
//
//		TArray4X4		T21 =
//		{
//			{ 1, 0, 0, 0, },
//			{ 0, 1, 0, 50, },
//			{ 0, 0, 1, 169,},
//			{ 0, 0, 0, 1, },
//		};
//
//		// L3
//		TArray2X4		link3 =
//		{
//			{0, 90.61, 43.72, 1, },
//			{ 0, -103.29, 43.72, 1, },
//		};
//
//		TArray4X4		C3 =
//		{
//			{1, 0, 0, 0, },
//			{ 0, cos(J[2]), -sin(J[2]), 0, },
//			{ 0, sin(J[2]), cos(J[2]), 0, },
//			{ 0, 0, 0, 1, },
//		};
//
//		TArray4X4		T32 =
//		{
//			{ 1, 0, 0, 0, },
//			{ 0, 1, 0, -0.15, },
//			{ 0, 0, 1, 421.27,},
//			{ 0, 0, 0, 1, },
//		};
//
//		// L4
//		TArray2X4		link4 =
//		{
//			{ 0, 0, 0, 1, },
//			{ 0, 399.2, 0, 1, },
//		};
//
//		TArray4X4		C4 =
//		{
//			{ cos(J[3]), 0, sin(J[3]), 0, },
//			{ 0, 1, 0, 0, },
//			{ -sin(J[3]), 0, cos(J[3]), 0, },
//			{ 0, 0, 0, 1, },
//		};
//
//		TArray4X4		T43 =
//		{
//			{ 1, 0, 0, 0, },
//			{ 0, 1, 0, 90.61, },
//			{ 0, 0, 1, 43.72,},
//			{ 0, 0, 0, 1, },
//		};
//
//		// L5
//		TArray2X4		link5 =
//		{
//			{ 0, 92.7, 0, 1, },
//			{ 0, -46.07, 0, 1, },
//		};
//
//		TArray4X4		C5 =
//		{
//			{1, 0, 0, 0, },
//			{ 0, cos(J[4]), -sin(J[4]), 0, },
//			{ 0, sin(J[4]), cos(J[4]), 0, },
//			{ 0, 0, 0, 1, },
//		};
//
//		TArray4X4		T54 =
//		{
//			{ 1, 0, 0, 0, },
//			{ 0, 1, 0, 349, },
//			{ 0, 0, 1, 0,},
//			{ 0, 0, 0, 1, },
//		};
//
//		// L6
//		TArray2X4		link6 =
//		{
//			{ 0,0, 0, 1, },
//			{ 0,0, 0, 1, },
//		};
//
//		TArray4X4		C6 =
//		{
//			{ cos(J[5]), 0, sin(J[5]), 0, },
//			{ 0, 1, 0, 0, },
//			{ -sin(J[5]), 0, cos(J[5]), 0, },
//			{ 0, 0, 0, 1, },
//
//		};
//
//		TArray4X4		T65 =
//		{
//			{ 1, 0, 0, 0, },
//			{ 0, 1, 0, 92.63, },
//			{ 0, 0, 1, 0,},
//			{ 0, 0, 0, 1, },
//		};
//
//		/* Link0 */
//
//		ArrayMul(T10, C1, tmpC);
//		ArrayMul(tmpC, link1, link1);
//		/* Link2 */
//		ArrayMul(T21, C2, tmp);
//		ArrayMul(tmpC, tmp, tmpC);
//		ArrayMul(tmpC, link2, link2);
//		/* Link3 */
//		ArrayMul(T32, C3, tmp);
//		ArrayMul(tmpC, tmp, tmpC);
//		ArrayMul(tmpC, link3, link3);
//		/* Link4 */
//		ArrayMul(T43, C4, tmp);
//		ArrayMul(tmpC, tmp, tmpC);
//		ArrayMul(tmpC, link4, link4);
//		/* Link5 */
//		ArrayMul(T54, C5, tmp);
//		ArrayMul(tmpC, tmp, tmpC);
//		ArrayMul(tmpC, link5, link5);
//		/* Link6 */
//		ArrayMul(T65, C6, tmp);
//		ArrayMul(tmpC, tmp, tmpC);
//		ArrayMul(tmpC, link6, link6);
//
//		memcpy(&link[0], &link0, sizeof(link0));
//		memcpy(&link[1], &link1, sizeof(link1));
//		memcpy(&link[2], &link2, sizeof(link2));
//		memcpy(&link[3], &link3, sizeof(link3));
//		memcpy(&link[4], &link4, sizeof(link4));
//		memcpy(&link[5], &link5, sizeof(link5));
//		memcpy(&link[6], &link6, sizeof(link6));
//
//		float       sjMax = 0.0;
//		for (int cnt = 0; cnt < CountOfArray(sj); cnt++) {
//			sj[cnt] = 0.0;
//			sj01[cnt] = false;
//		}
//		for (int a = 0; a < CountOfArray(link); a++) {
//			T3DValue    P0, P1;
//			T3DValue	V0;
//			float		d;
//			float		od, odTmp;
//			P1.x = link[a][1][0], P1.y = link[a][1][1], P1.z = link[a][1][2];
//
//			P0.x = link[a][0][0], P0.y = link[a][0][1], P0.z = link[a][0][2];
//
//			V0 = dectectPt[cubeZ][cubeY][cubeX];
//
//
//			d = Norm(Cross(Sub(P1, P0), Sub(V0, P0))) / Norm(Sub(P1, P0));
//			/*  */
//			od = -100000;
//
//			if (d <= (Resolution + r[a])) {
//
//				if (odTmp = ((Dot(Sub(V0, P1), Sub(P0, P1)) / Norm(Sub(P1, P0))) - Norm(Sub(P1, P0))), odTmp > 0) {
//
//					od = odTmp;
//
//				}
//				else if (odTmp = ((Dot(Sub(V0, P0), Sub(P1, P0)) / Norm(Sub(P1, P0))) - Norm(Sub(P1, P0))), odTmp > 0) {
//
//					od = odTmp;
//				}
//
//				/*  */
//				if (d <= r[a]
//					&& od > Resolution) {
//					sj[a] = (Resolution / od) * 0.5;
//					sj01[a] = false;
//					continue;
//				}
//
//				/*  */
//				if (d <= r[a]
//					&& od <= Resolution
//					&& od > 0) {
//					float	maxsafe = 1 + ((d / (Resolution + r[a])) * (0.5 - 1));
//
//
//					sj[a] = maxsafe + (od / (Resolution)) * (0.5 - maxsafe);
//					sj01[a] = true;
//					continue;
//				}
//				/*  */
//				if (od > 0
//					&& d > r[a]
//					&& (sqrt(pow((d - r[a]), 2) + pow(od, 2)) > Resolution)) {
//					sj[a] = (Resolution / sqrt(pow((d - r[a]), 2) + pow(od, 2))) * 0.5;
//					sj01[a] = false;
//					continue;
//				}
//
//				/*  */
//				if (od > 0
//					&& d > r[a]
//					&& (sqrt(pow((d - r[a]), 2) + pow(od, 2)) <= Resolution)) {
//					float	maxsafe = (1 + ((r[a] / (Resolution + r[a])) * (0.5 - 1)));
//
//					sj[a] = maxsafe + (sqrt(pow((d - r[a]), 2) + pow(od, 2)) / (Resolution)) * (0.5 - maxsafe);
//
//					sj01[a] = true;
//					continue;
//				}
//
//				/*  */
//				sj[a] = 1 + ((d / (Resolution + r[a])) * (0.5 - 1));
//				sj01[a] = true;
//
//			}
//			else {
//				/*  */
//				sj01[a] = false;
//
//				/*  */
//				if (odTmp = ((Dot(Sub(V0, P1), Sub(P0, P1)) / Norm(Sub(P1, P0))) - Norm(Sub(P1, P0))), odTmp > 0) {
//					od = odTmp;
//				}
//				else if (odTmp = ((Dot(Sub(V0, P0), Sub(P1, P0)) / Norm(Sub(P1, P0))) - Norm(Sub(P1, P0))), odTmp > 0) {
//					od = odTmp;
//				}
//
//				/*  */
//				if (od > 0) {
//
//					sj[a] = (Resolution / sqrt(pow((d - r[a]), 2) + pow(od, 2))) * 0.5;
//
//				}
//				else {
//
//					sj[a] = (((Resolution) / (d - r[a]))) * 0.5;
//				}
//			}
//		}
//
//		for (int cnt = 0; cnt < CountOfArray(sj); cnt++) {
//			if (!cnt) {
//				sjMax = sj[0];
//			}
//			else if (sj[cnt] > sjMax) {
//				sjMax = sj[cnt];
//			}
//		}
//		output[f][cubeZ][cubeY][cubeX] = sjMax;
//
//	}
//	return output;
//}
//std::vector<float> NewtonOptimize::RobotArmJ(cv::Mat PathData, int cubeX, int cubeY, int cubeZ)
//{
//	static const double	r[] = { 112.935,107.26,126.5,84.63,71.77,52.74, };
//	float	    Resolution = sqrt(((RESOLUTION / 2) * (RESOLUTION / 2)) * 3);
//	TArray4X4	tmp, tmpC;
//	TArray2X4	link[7];
//
//	float		sj[CountOfArray(link)];
//	bool		sj01[CountOfArray(sj)];
//	float       C[6];
//	vector<string> row;
//	//string line, word;
//	auto output = std::vector<std::vector<std::vector<std::vector<float>>>>(PathData.rows, std::vector<std::vector<std::vector<float>>>(CountOfArray(box), std::vector<std::vector<float>>(CountOfArray(box[0]), std::vector<float>(CountOfArray(box[0][0])))));
//	vector<float> result;
//	for (int f = 0; f < PathData.rows; f++)
//	{
//		for (int j = 0; j < PathData.row(f).cols; j++)
//		{
//			C[j] = PathData.at<float>(f, j);
//		}
//		float	J[] =
//		{
//			(((C[0]) / 180.0) * M_PI),
//			(((C[1]) / 180.0) * M_PI),
//			(((C[2]) / 180.0) * M_PI),
//			(((C[3]) / 180.0) * M_PI),
//			(((C[4]) / 180.0) * M_PI),
//			(((C[5]) / 180.0) * M_PI),
//		};
//
//		TArray2X4	link0 =
//		{
//			{ 0, 0, 0, 1, },
//			{ 0, 0, 190, 1, },
//		};
//
//		// L1
//		TArray2X4	link1 =
//		{
//			{ 0, 0, 0, 1, },
//			{ 0, 71.72, 242.41, 1, },
//		};
//
//		TArray4X4	C1 =                      //��ġ����ۡ������
//		{
//			{ cos(J[0]), -sin(J[0]), 0, 0, },
//			{ sin(J[0]), cos(J[0]), 0, 0, },
//			{ 0, 0, 1, 0, },
//			{ 0, 0, 0, 1, },
//		};
//
//		TArray4X4	T10 =                     //��ġ��������������ŷ����ٯ
//		{
//			{ 1, 0, 0, 0, },
//			{ 0, 1, 0, 0, },
//			{ 0, 0, 1, 190,},
//			{ 0, 0, 0, 1, },
//		};
//
//		// L2
//		TArray2X4		link2 =
//		{
//			{ 0, 0.02, -76.55, 1, },
//			{ 0, -0.15, 480.47, 1, },
//		};
//
//		TArray4X4		C2 =
//		{
//			{1, 0, 0, 0, },
//			{ 0, cos(J[1]), -sin(J[1]), 0, },
//			{ 0, sin(J[1]), cos(J[1]), 0, },
//			{ 0, 0, 0, 1, },
//		};
//
//		TArray4X4		T21 =
//		{
//			{ 1, 0, 0, 0, },
//			{ 0, 1, 0, 50, },
//			{ 0, 0, 1, 169,},
//			{ 0, 0, 0, 1, },
//		};
//
//		// L3
//		TArray2X4		link3 =
//		{
//			{0, 90.61, 43.72, 1, },
//			{ 0, -103.29, 43.72, 1, },
//		};
//
//		TArray4X4		C3 =
//		{
//			{1, 0, 0, 0, },
//			{ 0, cos(J[2]), -sin(J[2]), 0, },
//			{ 0, sin(J[2]), cos(J[2]), 0, },
//			{ 0, 0, 0, 1, },
//		};
//
//		TArray4X4		T32 =
//		{
//			{ 1, 0, 0, 0, },
//			{ 0, 1, 0, -0.15, },
//			{ 0, 0, 1, 421.27,},
//			{ 0, 0, 0, 1, },
//		};
//
//		// L4
//		TArray2X4		link4 =
//		{
//			{ 0, 0, 0, 1, },
//			{ 0, 399.2, 0, 1, },
//		};
//
//		TArray4X4		C4 =
//		{
//			{ cos(J[3]), 0, sin(J[3]), 0, },
//			{ 0, 1, 0, 0, },
//			{ -sin(J[3]), 0, cos(J[3]), 0, },
//			{ 0, 0, 0, 1, },
//		};
//
//		TArray4X4		T43 =
//		{
//			{ 1, 0, 0, 0, },
//			{ 0, 1, 0, 90.61, },
//			{ 0, 0, 1, 43.72,},
//			{ 0, 0, 0, 1, },
//		};
//
//		// L5
//		TArray2X4		link5 =
//		{
//			{ 0, 92.7, 0, 1, },
//			{ 0, -46.07, 0, 1, },
//		};
//
//		TArray4X4		C5 =
//		{
//			{1, 0, 0, 0, },
//			{ 0, cos(J[4]), -sin(J[4]), 0, },
//			{ 0, sin(J[4]), cos(J[4]), 0, },
//			{ 0, 0, 0, 1, },
//		};
//
//		TArray4X4		T54 =
//		{
//			{ 1, 0, 0, 0, },
//			{ 0, 1, 0, 349, },
//			{ 0, 0, 1, 0,},
//			{ 0, 0, 0, 1, },
//		};
//
//		// L6
//		TArray2X4		link6 =
//		{
//			{ 0,0, 0, 1, },
//			{ 0,0, 0, 1, },
//		};
//
//		TArray4X4		C6 =
//		{
//			{ cos(J[5]), 0, sin(J[5]), 0, },
//			{ 0, 1, 0, 0, },
//			{ -sin(J[5]), 0, cos(J[5]), 0, },
//			{ 0, 0, 0, 1, },
//
//		};
//
//		TArray4X4		T65 =
//		{
//			{ 1, 0, 0, 0, },
//			{ 0, 1, 0, 92.63, },
//			{ 0, 0, 1, 0,},
//			{ 0, 0, 0, 1, },
//		};
//
//		/* Link0 */
//
//		ArrayMul(T10, C1, tmpC);
//		ArrayMul(tmpC, link1, link1);
//		/* Link2 */
//		ArrayMul(T21, C2, tmp);
//		ArrayMul(tmpC, tmp, tmpC);
//		ArrayMul(tmpC, link2, link2);
//		/* Link3 */
//		ArrayMul(T32, C3, tmp);
//		ArrayMul(tmpC, tmp, tmpC);
//		ArrayMul(tmpC, link3, link3);
//		/* Link4 */
//		ArrayMul(T43, C4, tmp);
//		ArrayMul(tmpC, tmp, tmpC);
//		ArrayMul(tmpC, link4, link4);
//		/* Link5 */
//		ArrayMul(T54, C5, tmp);
//		ArrayMul(tmpC, tmp, tmpC);
//		ArrayMul(tmpC, link5, link5);
//		/* Link6 */
//		ArrayMul(T65, C6, tmp);
//		ArrayMul(tmpC, tmp, tmpC);
//		ArrayMul(tmpC, link6, link6);
//
//		memcpy(&link[0], &link0, sizeof(link0));
//		memcpy(&link[1], &link1, sizeof(link1));
//		memcpy(&link[2], &link2, sizeof(link2));
//		memcpy(&link[3], &link3, sizeof(link3));
//		memcpy(&link[4], &link4, sizeof(link4));
//		memcpy(&link[5], &link5, sizeof(link5));
//		memcpy(&link[6], &link6, sizeof(link6));
//
//		float       sjMax = 0.0;
//		for (int cnt = 0; cnt < CountOfArray(sj); cnt++) {
//			sj[cnt] = 0.0;
//			sj01[cnt] = false;
//		}
//		for (int a = 0; a < CountOfArray(link); a++) {
//			T3DValue    P0, P1;
//			T3DValue	V0;
//			float		d;
//			float		od, odTmp;
//			P1.x = link[a][1][0], P1.y = link[a][1][1], P1.z = link[a][1][2];
//
//			P0.x = link[a][0][0], P0.y = link[a][0][1], P0.z = link[a][0][2];
//
//			V0 = dectectPt[cubeZ][cubeY][cubeX];
//
//
//			d = Norm(Cross(Sub(P1, P0), Sub(V0, P0))) / Norm(Sub(P1, P0));
//			/*  */
//			od = -100000;
//
//			if (d <= (Resolution + r[a])) {
//
//				if (odTmp = ((Dot(Sub(V0, P1), Sub(P0, P1)) / Norm(Sub(P1, P0))) - Norm(Sub(P1, P0))), odTmp > 0) {
//
//					od = odTmp;
//
//				}
//				else if (odTmp = ((Dot(Sub(V0, P0), Sub(P1, P0)) / Norm(Sub(P1, P0))) - Norm(Sub(P1, P0))), odTmp > 0) {
//
//					od = odTmp;
//				}
//
//				/*  */
//				if (d <= r[a]
//					&& od > Resolution) {
//					sj[a] = (Resolution / od) * 0.5;
//					sj01[a] = false;
//					continue;
//				}
//
//				/*  */
//				if (d <= r[a]
//					&& od <= Resolution
//					&& od > 0) {
//					float	maxsafe = 1 + ((d / (Resolution + r[a])) * (0.5 - 1));
//
//
//					sj[a] = maxsafe + (od / (Resolution)) * (0.5 - maxsafe);
//					sj01[a] = true;
//					continue;
//				}
//				/*  */
//				if (od > 0
//					&& d > r[a]
//					&& (sqrt(pow((d - r[a]), 2) + pow(od, 2)) > Resolution)) {
//					sj[a] = (Resolution / sqrt(pow((d - r[a]), 2) + pow(od, 2))) * 0.5;
//					sj01[a] = false;
//					continue;
//				}
//
//				/*  */
//				if (od > 0
//					&& d > r[a]
//					&& (sqrt(pow((d - r[a]), 2) + pow(od, 2)) <= Resolution)) {
//					float	maxsafe = (1 + ((r[a] / (Resolution + r[a])) * (0.5 - 1)));
//
//					sj[a] = maxsafe + (sqrt(pow((d - r[a]), 2) + pow(od, 2)) / (Resolution)) * (0.5 - maxsafe);
//
//					sj01[a] = true;
//					continue;
//				}
//
//				/*  */
//				sj[a] = 1 + ((d / (Resolution + r[a])) * (0.5 - 1));
//				sj01[a] = true;
//
//			}
//			else {
//				/*  */
//				sj01[a] = false;
//
//				/*  */
//				if (odTmp = ((Dot(Sub(V0, P1), Sub(P0, P1)) / Norm(Sub(P1, P0))) - Norm(Sub(P1, P0))), odTmp > 0) {
//					od = odTmp;
//				}
//				else if (odTmp = ((Dot(Sub(V0, P0), Sub(P1, P0)) / Norm(Sub(P1, P0))) - Norm(Sub(P1, P0))), odTmp > 0) {
//					od = odTmp;
//				}
//
//				/*  */
//				if (od > 0) {
//
//					sj[a] = (Resolution / sqrt(pow((d - r[a]), 2) + pow(od, 2))) * 0.5;
//
//				}
//				else {
//
//					sj[a] = (((Resolution) / (d - r[a]))) * 0.5;
//				}
//			}
//
//		}
//		//cout << "5 "<<sj[5] << endl;
//		//cout << PathData.row(f) << endl;
//
//		for (int j = 0; j < JointCount; j++) {
//			result.push_back(sj[j]);
//		}
//		for (int cnt = 0; cnt < CountOfArray(sj); cnt++) {
//			if (!cnt) {
//				sjMax = sj[0];
//			}
//			else if (sj[cnt] > sjMax) {
//				sjMax = sj[cnt];
//
//			}
//		}
//		output[f][cubeZ][cubeY][cubeX] = sjMax;
//	}
//	return result;
//}
//void
//NewtonOptimize::ArrayMul(TArray4X4 a, TArray4X4 b, TArray4X4& rVal)
//{
//	TArray4X4   result;
//
//	for (int i = 0; i < 4; i++) {
//		for (int j = 0; j < 4; j++) {
//			float		sum = 0;
//
//			for (int k = 0; k < 4; k++) {
//				sum += (a[i][k] * b[k][j]);
//			}
//
//			result[i][j] = sum;
//		}
//	}
//	memcpy(&rVal, &result, sizeof(rVal));
//}
//void
//NewtonOptimize::ArrayMul(TArray4X4 a, TArray2X4 b, TArray2X4& rVal)
//{
//	TArray4X4   result;
//
//
//	for (int i = 0; i < 4; i++) {
//		for (int j = 0; j < 2; j++) {
//			float		sum = 0;
//
//			for (int k = 0; k < 4; k++) {
//				sum += (a[i][k] * b[j][k]);
//			}
//
//			result[j][i] = sum;
//		}
//	}
//	memcpy(&rVal, &result, sizeof(rVal));
//}
//void
//NewtonOptimize::ArrayMul(TArray4X4 a, const float* bPtr, float* rVal)
//{
//	if ((bPtr != NULL)
//		&& (rVal != NULL)) {
//		TArray4     result;
//
//		for (int i = 0; i < 4; i++) {
//			float		sum = 0;
//			float* aPtr = a[i];
//
//			for (int k = 0; k < 4; k++, aPtr++) {
//				sum += (*aPtr * bPtr[k]);
//			}
//
//			result[i] = sum;
//		}
//		memcpy(rVal, &result, sizeof(result));
//	}
//}
//float
//NewtonOptimize::Norm(T3DValue a)
//{
//	return sqrt((a.x * a.x) + (a.y * a.y) + (a.z * a.z));
//}
//
//NewtonOptimize::T3DValue NewtonOptimize::Cross(T3DValue a, T3DValue b)
//{
//	T3DValue	c;
//
//	c.x = (a.y * b.z) - (a.z * b.y);
//	c.y = (a.z * b.x) - (a.x * b.z);
//	c.z = (a.x * b.y) - (a.y * b.x);
//
//	return c;
//}
//float
//NewtonOptimize::Dot(T3DValue a, T3DValue b)
//{
//	return (a.x * b.x) + (a.y * b.y) + (a.z * b.z);
//}
//NewtonOptimize::T3DValue NewtonOptimize::Sub(T3DValue a, T3DValue b)
//{
//	a.x -= b.x;
//	a.y -= b.y;
//	a.z -= b.z;
//
//	return a;
//}
//cv::Mat NewtonOptimize::pathcreate(cv::Mat start, cv::Mat end, int sampleRange) {
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
//
//
