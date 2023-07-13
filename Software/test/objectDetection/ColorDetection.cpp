#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/core.hpp>
#include "opencv2/opencv.hpp"
#include <iostream>
using namespace cv;
using namespace std;
Mat src_large, src, erosion_dst, dilation_dst;

int erosion_elem = 0;
int erosion_size = 0;
int dilation_elem = 0;
int dilation_size = 0;
int const max_elem = 2;
int const max_kernel_size = 100;
void Erosion( int, void* );
void Dilation( int, void* );

int morphology(Mat src)
{
  namedWindow( "Erosion Demo", WINDOW_AUTOSIZE );
  namedWindow( "Dilation Demo", WINDOW_AUTOSIZE );
  moveWindow( "Dilation Demo", src.cols, 0 );
  createTrackbar( "Element:\n 0: Rect \n 1: Cross \n 2: Ellipse", "Erosion Demo",
          &erosion_elem, max_elem,
          Erosion );
  createTrackbar( "Kernel size:\n 2n +1", "Erosion Demo",
          &erosion_size, max_kernel_size,
          Erosion );
  createTrackbar( "Element:\n 0: Rect \n 1: Cross \n 2: Ellipse", "Dilation Demo",
          &dilation_elem, max_elem,
          Dilation );
  createTrackbar( "Kernel size:\n 2n +1", "Dilation Demo",
          &dilation_size, max_kernel_size,
          Dilation );
  Erosion( 0, 0 );
  Dilation( 0, 0 );
  // waitKey(0);
  return 0;
}
void Erosion( int, void* )
{
  int erosion_type = 0;
  if( erosion_elem == 0 ){ erosion_type = MORPH_RECT; }
  else if( erosion_elem == 1 ){ erosion_type = MORPH_CROSS; }
  else if( erosion_elem == 2) { erosion_type = MORPH_ELLIPSE; }
  Mat element = getStructuringElement( erosion_type,
                       Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                       Point( erosion_size, erosion_size ) );
  erode( src, erosion_dst, element );
  imshow( "Erosion Demo", erosion_dst );
}
void Dilation( int, void* )
{
  int dilation_type = 0;
  if( dilation_elem == 0 ){ dilation_type = MORPH_RECT; }
  else if( dilation_elem == 1 ){ dilation_type = MORPH_CROSS; }
  else if( dilation_elem == 2) { dilation_type = MORPH_ELLIPSE; }
  Mat element = getStructuringElement( dilation_type,
                       Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                       Point( dilation_size, dilation_size ) );
  dilate( src, dilation_dst, element );
  imshow( "Dilation Demo", dilation_dst );
}

int histogramColorDetection(Mat src)
{

  vector<Mat> bgr_planes;
  split(src, bgr_planes);
  int histSize = 256;
  float range[] = {0, 256}; // the upper boundary is exclusive
  const float *histRange[] = {range};
  bool uniform = true, accumulate = false;
  Mat b_hist, g_hist, r_hist;
  calcHist(&bgr_planes[0], 1, 0, Mat(), b_hist, 1, &histSize, histRange, uniform, accumulate);
  calcHist(&bgr_planes[1], 1, 0, Mat(), g_hist, 1, &histSize, histRange, uniform, accumulate);
  calcHist(&bgr_planes[2], 1, 0, Mat(), r_hist, 1, &histSize, histRange, uniform, accumulate);
  int hist_w = 512, hist_h = 400;
  int bin_w = cvRound((double)hist_w / histSize);
  Mat histImage(hist_h, hist_w, CV_8UC3, Scalar(0, 0, 0));
  normalize(b_hist, b_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat());
  normalize(g_hist, g_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat());
  normalize(r_hist, r_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat());
  for (int i = 1; i < histSize; i++)
  {
    line(histImage, Point(bin_w * (i - 1), hist_h - cvRound(b_hist.at<float>(i - 1))),
         Point(bin_w * (i), hist_h - cvRound(b_hist.at<float>(i))),
         Scalar(255, 0, 0), 2, 8, 0);
    line(histImage, Point(bin_w * (i - 1), hist_h - cvRound(g_hist.at<float>(i - 1))),
         Point(bin_w * (i), hist_h - cvRound(g_hist.at<float>(i))),
         Scalar(0, 255, 0), 2, 8, 0);
    line(histImage, Point(bin_w * (i - 1), hist_h - cvRound(r_hist.at<float>(i - 1))),
         Point(bin_w * (i), hist_h - cvRound(r_hist.at<float>(i))),
         Scalar(0, 0, 255), 2, 8, 0);
  }
  namedWindow("Source image");
  imshow("Source image", src);
  imshow("calcHist Demo", histImage);
  // waitKey();
  return EXIT_SUCCESS;
}

Mat src_gray;
Mat dst, detected_edges;
int lowThreshold = 0;
const int max_lowThreshold = 100;
const int ration = 3;
const int kernel_size = 3;
const char *window_name = "Edge Map";

static void CannyThreshold(int, void *)
{
  blur(src_gray, detected_edges, Size(3, 3));
  Canny(detected_edges, detected_edges, lowThreshold, lowThreshold * ration, kernel_size);
  dst = Scalar::all(0);
  src.copyTo(dst, detected_edges);
  imshow(window_name, dst);
}

int cannyEdgeDetector(Mat src)
{
  dst.create(src.size(), src.type());
  cvtColor(src, src_gray, COLOR_BGR2GRAY);
  namedWindow(window_name, WINDOW_AUTOSIZE);
  createTrackbar("Min Threshold:", window_name, &lowThreshold, max_lowThreshold, CannyThreshold);
  CannyThreshold(0, 0);
  // waitKey(0);
  return 0;
}

int main(int argc, char **argv)
{
  CommandLineParser parser(argc, argv, "{@input | TestImage.jpg | input image}");
  src_large = imread(samples::findFile(parser.get<String>("@input")), IMREAD_COLOR);
  resize(src_large,src,Size(1000,800),1);
  if (src.empty())
  {
    std::cout << "Could not open or find the image!\n"<< std::endl;
    std::cout << "Usage: " << argv[0] << " <Input image>" << std::endl;
    return EXIT_FAILURE;
  }
  morphology(src);
  histogramColorDetection(src);
  cannyEdgeDetector(src);
  waitKey(0);
  return EXIT_SUCCESS;
}