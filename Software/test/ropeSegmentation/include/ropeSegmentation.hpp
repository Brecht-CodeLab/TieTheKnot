#ifndef DETECTION
#define DETECTION
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/core.hpp>
#include "opencv2/opencv.hpp"
// #include "opencv2/cudaarithm.hpp"
#include <math.h> 
#include <iostream>
#include <stdio.h>


using namespace cv;
using namespace std;

namespace Detection{
  pair<double, double> calculateMinMaxValues(Mat image, char * title);
  void createDifferentKernels(int nbOfKernels, Mat kernel, vector<Mat> * kernels, vector<Mat> * rotationMatrices);
  double convolution(Mat image, Mat kernel);
  void tryDifferentKernelOrientations(int currentKernelIndex, int wantedKernelsToTry, Mat imagePart ,vector<Mat> * kernels, vector<double> * convolutionResults);
  int convolutionMap(Mat image,double maxValueAppearence, double minValueAppearence, double thresholdFactorOnMaxValue,  vector<Mat> * kernels, int currentKernelIndex, vector<int> * initialPoint);
  void drawCircles(Mat image, vector<vector<int>> * points);
  void checkBestConvolutionResult(vector<double> * convolutionResults, double * bestResult, int * bestResultIndex);
  void ropeSegmentation( Mat image, vector<int> * initialPoint, int initialKernelIndex, vector<Mat> * kernels, vector<Mat> * rotationMatrices, vector<vector<int>> * segmentationPoints, int nbOfWantedKernelsToTry, Mat step);
}
using namespace Detection;

#endif