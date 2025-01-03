//
//#include <iostream>
//#include "test.h"
//#include <stdlib.h>
//#include <stdio.h>
//#include <opencv2/opencv.hpp>
//#include "NewtonUpdate.h"
////#include "Function.h"
//
//using namespace std;
//using namespace cv;
//cv::Mat pathcreate(cv::Mat start, cv::Mat end,int sampleRange) {
//	cv::Mat P1 = start;
//	cv::Mat P2 = end;
//
//	cv::Mat dis;
//
//	for (int i = 0; i < P1.cols; i++) {
//		dis.push_back(P1.col(i) - P2.col(i));
//		//std::cout << P1.col(i) - P2.col(i) << std::endl;
//
//	}
//
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
//		std::cout << pos << std::endl;
//		path.push_back(pos);
//	}
//
//	return path;
//
//}
//Mat Create(Mat pastRoute, Mat Routenow, int routeindex) {
//	Mat tmp;
//
//
//		tmp.push_back(pastRoute);
//	
//	for (int i = routeindex; i < Routenow.rows; i++) {
//		tmp.push_back(Routenow.row(i));
//	}
//
//	return tmp;
//}
//
//int main()
//{
//	////First Row is Min, Second Row is Max Range
//	////chrono::high_resolution_clock::time_point J1 = chrono::high_resolution_clock::now();
//
//	char filename[256];
//	FILE* outfile;
//	Mat JointRangeMatrix = (Mat_<float>(2, 6) << -165, -125, -55, -190, -115, -180, 			//各軸角度限制
//		165, 85, 185, 190, 115, 180);
//
//	//Ptr<ml::TrainData> data = ml::TrainData::loadFromCSV("C:/Users/ttuge/Desktop/人機協做期末展演資料/0317.csv", 0, -2, 0);//-2 0
//
//	Ptr<ml::TrainData> data = ml::TrainData::loadFromCSV("path/1116.csv", 0, -2, 0); //讀取路徑(可能會改
//	Mat RouteSample = data->getTrainSamples();
//
//
//	Mat DataOffset = Mat::zeros(1, RouteSample.cols, CV_32F);
//
//	copyMakeBorder(DataOffset, DataOffset, 0, RouteSample.rows - 1, 0, 0, BORDER_REFLECT); //角度偏移用(目前不會用到
//
//	RouteSample = RouteSample + DataOffset;
//
//	sprintf_s(filename, "./result/tmp.csv");
//	fopen_s(&outfile, filename, "w");
//	//NewtonOptimize PosOptimizer(JointRangeMatrix, RouteSample); //宣告物件
//	//vector<Mat> Path = PosOptimizer.RoutePlanning(RouteSample, 7 - 1, 10 - 1, 2 - 1); //執行路徑計算
//	//Mat tmp=Mat_<float>(0,6);
//	bool flag = true; int posCount = 1;
//	NewtonOptimize PosOptimizer(JointRangeMatrix, RouteSample);
//	while (flag) {
//		string a, b, c;
//		cout << "cube: ";
//		cin >> a >> b >> c;
//		std::string pathing = "result/box " + a;
//		pathing = pathing + "-";
//		pathing = pathing + b;
//		pathing = pathing + "-";
//		pathing = pathing + c;
//		cout << pathing << endl;
//
//		int x_ = stoi(a);
//		int y_ = stoi(b);
//		int z_ = stoi(c);
//		
//		vector<Mat> Path = PosOptimizer.RoutePlanning(x_ - 1, y_ - 1, z_ - 1); //執行路徑計算
//		//PosOptimizer.setCube(x_, y_, z_);
//		chrono::high_resolution_clock::time_point J1 = chrono::high_resolution_clock::now();
//		//vector<Mat> Path = PosOptimizer.RoutePlanning(x_ - 1, y_ - 1, z_ - 1); //執行路徑計算
//		chrono::high_resolution_clock::time_point J2 = chrono::high_resolution_clock::now();
//		//std::cout << chrono::duration_cast<chrono::milliseconds>(J2 - J1).count() << " ms" << endl;
//		for (int i = 0; i < Path.size(); i++) {
//			cout << Path[i] << endl;
//		}
//		for (int i = 1; i < Path.size(); i++) {
//			cout << i << Path.size() << endl;
//			cout << Path[i] << endl;
//			char yn = 'n';
//
//
//			if (i == 0) {
//				auto StartPos = Path.front();
//				cout << "front" << endl;
//				//for (int n = 0; n < StartPos.cols; n++) {
//				//	fprintf(outfile, "%f,", StartPos.at<float>(0, n));
//				//}
//				//fprintf(outfile, "\n");
//			}
//			//auto EndPos = RouteSample.row(TargetPos.back());
//			//cout << EndPos << endl;
//			//Mat LastPos = StartPos;
//			//PosOptimizer.setLastPos(StartPos);
//			//while(posCount < TargetPos.size() - 1) {
//			//	cout << posCount << endl;
//			//	chrono::high_resolution_clock::time_point J1 = chrono::high_resolution_clock::now();
//			//	auto SafePos = PosOptimizer.OptimizeSinglePos(RouteSample.row(TargetPos[posCount]), StartPos, EndPos);
//			//	chrono::high_resolution_clock::time_point J2 = chrono::high_resolution_clock::now();
//			//	std::cout << chrono::duration_cast<chrono::milliseconds>(J2 - J1).count() << " ms" << endl;
//			//	PosOptimizer.setLastPos(SafePos);
//			//	cout << SafePos << endl << PosOptimizer.CalcD(SafePos, true) << endl;
//			//	cout << PosOptimizer.getcubeX() << PosOptimizer.getcubeY() << PosOptimizer.getcubeZ() << endl;
//			//for (int j = 0; j < Path[i].cols; j++) {
//			//	fprintf(outfile, "%f,", Path[i].at<float>(0, j));
//			//}
//			//fprintf(outfile, "%\n");
//			cout << "new cube? yn: ";
//
//			std::cin >> yn;
//			if (yn == 'y' || i == Path.size()-1) {
//				//Mat sample;
//				//for (int j = 0; j < Path.size()-1; j++) {
//				//	
//				//	sample.push_back(pathcreate(Path[j+1],Path[j],20));
//				//	cout << j << endl;
//				//}
//				cout << "here" << endl;
//				//RouteSample = Create(sample, RouteSample, PosOptimizer.returnIndex());
//				//flag = false;
//				
//				break;
//			}
//
//			//if (posCount == TargetPos.size() - 2) {
//			//	flag = false;
//			//	//for (int i = 0; i < EndPos.cols; i++) {
//			//	//	fprintf(outfile, "%f,", EndPos.at<float>(0, i));
//			//	//}
//			//	_fcloseall;
//			//}
//			//posCount++;
//
//		//}
//			//if (i == Path.size() - 1) {
//			//	cout << "last" << endl;
//			//	//auto EndPos = Path.back();
//			//	flag = false;
//			//	//for (int m = 0; m < EndPos.cols; m++) {
//			//	//	fprintf(outfile, "%f,", EndPos.at<float>(0, m));
//			//	//}
//			//	_fcloseall;
//			//}
//		}
//
//	}
//
//
//
//	float lenght = 0;
//	//Ptr<ml::TrainData> re = ml::TrainData::loadFromCSV("result/dynamic.csv", 0, -2, 0); //讀取路徑(可能會改
//	//Mat RouteResult = re->getTrainSamples();
//	//for (int i = 0; i < RouteResult.rows; i++) {
//	//	cout << RouteResult.row(i) << endl;
//	//	if (i > 0) {
//	//		lenght += cv::norm(RouteResult.row(i), RouteResult.row(i - 1));
//	//	}
//	//}
//	//cout << lenght << endl;
//
//	return 0;
//}
//
