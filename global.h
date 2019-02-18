#ifndef INCLUDES_H
#define INCLUDES_H value

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <chrono>

#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <numeric>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;

#define windowDim 1000

#define windowName "video"
#define windowGray "gray"

//Usefull macros
#define FOR(i,n) for(int i = 0; i < n;i++)

enum Pattern{CHESSBOARD,CIRCLES_GRID,ASYMMETRIC_CIRCLES_GRID,RINGS_GRID};
enum modes{DETECT_MODE,CALIBRATION_MODE,UNDISTORTION_MODE};

//RING GRID FUNCTION
bool findRingsGridPattern(cv::Mat Input, cv::Size size, std::vector<cv::Point2f>& points, bool& isTracking, std::vector<cv::Point2f>& oldPoints);
// RING PATTERN DETECTION ADDITIONAL FUNCTIONS
vector<Point2f> getControlPoints(const vector<Point2f> & centers);
bool FindRingPattern(vector<Point2f> &probableCPs,cv::Mat & frame,int num_rows,int num_cols);


// Funciones Adicionales para la Matrix Intrinseca
void calcBoardCornerPositions(cv::Size size, float squareSize, std::vector<cv::Point3f> &corners, int patternType);
double computeReprojectionErrors(const std::vector< std::vector<cv::Point3f> >& objectPoints,
									const std::vector< std::vector<cv::Point2f> >& imagePoints,
									const std::vector<cv::Mat>& rvecs,const std::vector<cv::Mat>& tvecs,
									const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs,
									std::vector<float> & perFrameErrors);

//Funciones para Generar los Numeros Combinatorios
void printCombination(std::vector< std::vector<int> >& v, int arr[], int n, int r);
void combinationUtil(std::vector< std::vector<int> >& v, int arr[], std::vector<int> &data, int start, int end, int index, int r);
std::vector< std::vector<int> > GenerateCombinations(int n, int r);


// Otras Funciones
bool cmpx(Point2f a,Point2f b);
bool cmpy(Point2f a, Point2f b);
float dist(Point2f a, Point2f b);

float StandarDesviation(const std::vector<float> & values );
float SimpleAverage(const std::vector<float> & v);

//Calcula una linea cada cierta cantidad de puntos
float getAvgColinearityFromVector(const std::vector<cv::Point2f>& PointBuffer, cv::Size size);
float printAvgColinearity(const std::vector<float>& v);

/** TEMPLATES **/

//Funcion para imprimir los valores de un vector
template <typename T>
void VizVector(const std::vector<T>&v){
	cout << "[";
	FOR(i,v.size())
		cout << v[i] << ",";
	cout << "]\n";
}

/** FRONTO PARALLEL TRANSFORM **/
std::vector<cv::Point2f> extractCorners(std::vector<cv::Point2f>& v, cv::Size size);

std::vector<cv::Point2f> getFrontoParallelCorners(cv::Size imgSize, cv::Size patternSize);

vector<Point2f>  distortion(vector<Point2f> cp,const cv::Mat& intrinsics,const cv::Mat& dist_coeff );

std::vector<bool> calc_affection(const std::vector<cv::Point2f> & PointBuffer,const cv::Size & imgPixelSize, const cv::Size & GridSize );

std::vector<int> calc_BestFrameCombination(const std::vector< std::vector<bool> > &voAffections, int noImages);

#endif // INCLUDES_H
