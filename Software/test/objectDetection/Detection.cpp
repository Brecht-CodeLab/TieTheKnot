#include <detection.hpp>


class ObjectDetection
{
private:
  void erosion()
  {
    if (erosion_elem == 0){erosion_type = MORPH_RECT;}
    else if (erosion_elem == 1){erosion_type = MORPH_CROSS;}
    else if (erosion_elem == 2){erosion_type = MORPH_ELLIPSE;}
    Mat element = getStructuringElement(erosion_type,
                                        Size(2 * erosion_size + 1, 2 * erosion_size + 1),
                                        Point(erosion_size, erosion_size));
    erode(morph, eroded, element);
    if (img_show)
      imshow("Erosion", eroded);
  }
  void dilation()
  {
    imshow("Dilation", src);
    if (dilation_elem == 0){dilation_type = MORPH_RECT;}
    else if (dilation_elem == 1){dilation_type = MORPH_CROSS;}
    else if (dilation_elem == 2){dilation_type = MORPH_ELLIPSE;}
    Mat element = getStructuringElement(dilation_type,
                                        Size(2 * dilation_size + 1, 2 * dilation_size + 1),
                                        Point(dilation_size, dilation_size));
    dilate(src, dilated, element);
    if (img_show)
      imshow("Dilation", dilated);
  }
  void morphologyCleaning()
  {
    int operation = morph_operator + 2;
    Mat element = getStructuringElement(morph_elem,
                                        Size(2 * morph_size + 1, 2 * morph_size + 1),
                                        Point(morph_size, morph_size));
    morphologyEx(dilated, morph, operation, element);
    if (img_show)
      imshow("Morph Ex", morph);
  }

public:
  // VARIABLES
  int erosion_elem = 2;
  int erosion_type = 0;
  int erosion_size = 6;
  int dilation_elem = 2;
  int dilation_type = 0;
  int dilation_size = 7;
  int morph_elem = 0;
  int morph_size = 6;
  int morph_operator = 0;
  bool img_show = true;
  Mat src, eroded, dilated, morph;
  // FUNCTIONS
  int morphology()
  {
    namedWindow("Erosion", WINDOW_AUTOSIZE);
    namedWindow("Dilation", WINDOW_AUTOSIZE);
    namedWindow("Morph Ex", WINDOW_AUTOSIZE);
    dilation();
    morphologyCleaning();
    erosion();
    return 0;
  }
  int histogramColorDetection(Mat src)
  {
    return 0;
  }
  int cannyEdgeDetector(Mat src)
  {
    return 0;
  }
};





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
  // namedWindow("Source image");
  // imshow("Source image", src);
  // imshow("calcHist Demo", histImage);
  // waitKey();
  return EXIT_SUCCESS;
}










double distance(const Vec2i p1, const Vec2i p2)
{
  return sqrt(pow(p1[0]-p2[0],2)+pow(p1[1]-p2[1],2));
}



int determineClosest(const Vec2i point, vector<Vec2i> list)
{
  double shortestDistance = distance(point,list[0]);
  int index = 0;
  for (size_t i=0; i<list.size(); i++)
  {
    double nextDist = distance(point,list[i]);
    if(nextDist < shortestDistance)
    {
      shortestDistance = nextDist;
      index = i;
    }
  }
  return index;
}

vector<Vec2i> sortListOnDistance(vector<Vec2i> list)
{
  vector<Vec2i> cpyList = list;
  vector<Vec2i> sortedList;
  sortedList.insert(sortedList.end(),1,cpyList[0]);
  cpyList.erase(cpyList.begin());
  for(size_t i=0; i<list.size()-1; i++)
  {
    int indexClosest = determineClosest(sortedList[i],cpyList);
    // std::cout << "Closest Index: " << indexClosest << endl;
    sortedList.insert(sortedList.end(),1,cpyList[indexClosest]);
    cpyList.erase(cpyList.begin() + indexClosest);
  }
  return sortedList;
}


vector<Vec2i> middleSegments(vector<Vec2i> sortedList, Mat img)
{
  vector<Vec2i> middleList;
  Vec2i newPoint;
  for(size_t i=0; i<sortedList.size()-1; i++)
  {
    newPoint[0] = (sortedList[i][0] + sortedList[i+1][0])/2;
    newPoint[1] = (sortedList[i][1] + sortedList[i+1][1])/2;

    cout << "New Point " << i << " : " << newPoint << endl;

    int pix = (int)img.at<uchar>(newPoint[0],newPoint[1]);
    cout << "Pixel Value: " << pix << endl;
    if(pix == 0)
    {
      // This means the middlepoint still is located ON the rope
      middleList.insert(middleList.end(),1,newPoint);
      Point center;
      int radius = 5;
      // middle_sgm[i][1] = v[3]-v[1];
      center.x = newPoint[0];
      center.y = newPoint[1];
      circle( img, center, radius, Scalar( 0, 0, 255), FILLED, LINE_8);
      cout << "Valid point" << endl;
    }
    else
    {
      cout << "Not Valid Point" << endl;
    }
  }
  imshow("Middle points",img);
  imwrite("MiddlePoints.jpg", img);
  return middleList;
}



int main(int argc, char **argv)
{
  Mat original, original_large,extractedR, extractedG, extractedB, fullImageHSV, tmpMask, redFiltered1, redFiltered2, greenFiltered, morphOpen, morphClose, eroded, dilated;
  CommandLineParser parser(argc, argv, "{@input | orangeRope3.jpg | input image}");
  original = imread(samples::findFile(parser.get<String>("@input")), IMREAD_COLOR);
  resize(original, original, Size((int)original.size().width/2, (int)original.size().height/2), 1);
  if (original.empty())
  {
    std::cout << "Could not open or find the image!\n"
              << std::endl;
    std::cout << "Usage: " << argv[0] << " <Input image>" << std::endl;
    return EXIT_FAILURE;
  }
  namedWindow("Initial Picture", WINDOW_AUTOSIZE);
  imshow("Initial Picture", original);
  imwrite("InitialPicture.jpg", original);

  // ObjectDetection objectDetection;
  // objectDetection.src = original;
  // objectDetection.morphology();

  extractChannel(original,extractedB,0);
  extractChannel(original,extractedG,1);
  extractChannel(original,extractedR,2);
  // namedWindow("Extracted Blue", WINDOW_AUTOSIZE);
  // namedWindow("Extracted Green", WINDOW_AUTOSIZE);
  // namedWindow("Extracted Red", WINDOW_AUTOSIZE);
  // imshow("Extracted Red",extractedR);
  // imshow("Extracted Green",extractedG);
  // imshow("Extracted Blue",extractedB);

  // histogramColorDetection(original);

  cvtColor(original,fullImageHSV, COLOR_BGR2HSV);
  // imshow("HSV",fullImageHSV);
  inRange(fullImageHSV, Scalar(0,60,45), Scalar(10, 255, 255), redFiltered1);
  inRange(fullImageHSV, Scalar(160,60,45), Scalar(180, 255, 255), redFiltered2);
  // inRange(fullImageHSV, Scalar(40,40,40), Scalar(70, 255, 255), greenFiltered);
  bitwise_or(redFiltered1, redFiltered2, redFiltered1);
  imshow("TMP Mask Red", redFiltered1);
  // imshow("TMP Mask Green", greenFiltered);


  

  // int morph_elem = 0;
  // int morph_size = 5;
  // int morph_operator = 0;
  // int operation = morph_operator + 2;
  // Mat elementMorph = getStructuringElement(morph_elem,
  //                                       Size(2 * morph_size + 1, 2 * morph_size + 1),
  //                                       Point(morph_size, morph_size));
  // morphologyEx(redFiltered, morphOpen, MORPH_OPEN, elementMorph);
  // imshow("Morph Open", morphOpen);

  // Mat elementErodeCross = getStructuringElement(MORPH_CROSS,Size(5,5));
  // Mat skeleton(redFiltered.size(), CV_8UC1, cv::Scalar(0));
  // morphologyEx(redFiltered1, morphClose, MORPH_CLOSE, elementMorph);
  // imshow("Morph Closed", morphClose);
  // bitwise_not(morphClose,morphClose);
  // bitwise_and(redFiltered,morphClose,morphClose);
  // bitwise_or(skeleton,morphClose,skeleton);
  // imshow("First Eroded", morphClose);
  // imshow("Skeleton", skeleton);


  int erosion_elem = 4;
  int erosion_type = 0;
  int erosion_size = 10;
  Mat elementErosion = getStructuringElement(erosion_type,
                                        Size(2 * erosion_size + 1, 2 * erosion_size + 1),
                                        Point(erosion_size, erosion_size));

  // erode(eroded,eroded,elementErodeCross);
  // imshow("Skeleton", eroded); 
  int dilation_elem = 2;
  int dilation_type = 0;
  int dilation_size = 10;
  Mat elementDilation = getStructuringElement(dilation_type,
                                        Size(2 * dilation_size + 1, 2 * dilation_size + 1),
                                        Point(dilation_size, dilation_size));







  // Otsu
  Mat otsu, gaussian, tozero, finalimg;
  Mat hsv_channels[3];
  cv::split( fullImageHSV, hsv_channels );
  Mat gray_col = hsv_channels[2];
  // Mat gray_col;
  // cvtColor(original,gray_col, COLOR_BGR2GRAY);
  // imshow("HSV to gray", hsv_channels[2]);
  // cvtColor(fullImageHSV,gray_col, COLOR_BGR2GRAY);
  GaussianBlur(gray_col,gaussian, Size(3,3),0);
  imshow("Gauss", gaussian);
  threshold(gaussian, otsu, 127, 255, THRESH_BINARY);
  imshow("OTSU", otsu);
  imwrite("Otsu.jpg", otsu);

  erode(otsu, eroded, elementErosion);
  // imshow("Eroded Red", eroded);

  dilate(eroded, dilated, elementDilation);
  // imshow("Dilated Red", dilated);

  Mat ropeBlob;
  bitwise_not(dilated,dilated);
  bitwise_and(redFiltered1,dilated,ropeBlob);
  bitwise_and(otsu,ropeBlob,ropeBlob);
  imshow("AND NOT", ropeBlob);

  erosion_size = 2;
  elementErosion = getStructuringElement(erosion_type,
                                        Size(2 * erosion_size + 1, 2 * erosion_size + 1),
                                        Point(erosion_size, erosion_size));
  erode(ropeBlob, eroded, elementErosion);
  imshow("Eroded 2", eroded);
  dilation_size = 5;
  elementDilation = getStructuringElement(dilation_type,
                                        Size(2 * dilation_size + 1, 2 * dilation_size + 1),
                                        Point(dilation_size, dilation_size));
  Mat ropeThick;
  dilate(eroded,ropeThick,elementDilation);


  Mat contoursImg = ropeThick.clone();
  // bitwise_not(contoursImg,contoursImg);
  vector<vector<Point>> contours;
  vector<Vec4i> hierarchy;
  int edgeThresh = 1;

  Canny(ropeBlob,contoursImg,edgeThresh,edgeThresh,3);
  imshow("Canny", contoursImg);
  // findContours(contoursImg,contours,hierarchy,RETR_EXTERNAL, CHAIN_APPROX_NONE);
  // drawContours(contoursImg, contours, -1, (0,255,0), 6);
  // drawContours(contoursImg, contours, -1, (0,255,0), 6);
  // imshow("Contours", contoursImg);


  Ptr<LineSegmentDetector> lsd = createLineSegmentDetector(LSD_REFINE_STD);
  vector<Vec4i> lines_std;
  vector<Vec2i> middle_sgm;
  lsd->detect(eroded, lines_std);
  bitwise_or(eroded, contoursImg, eroded);
  // lsd->drawSegments(eroded, lines_std);

  
  // imshow("line segment",eroded);

  int radius = 5;
  int len_vec = lines_std.size();
  // cout << "Number of segments: " << len_vec << endl;

  // for (size_t i=0; i<lines_std.size(); i++) {
  //     Vec4i lineSegment = lines_std[i];
  //     Vec2i v2 = {(lineSegment[2]+lineSegment[0])/2, (lineSegment[3]+lineSegment[1])/2};
  //     middle_sgm.insert(middle_sgm.end(),1,v2);

  //     Point center;
  //     // middle_sgm[i][1] = v[3]-v[1];
  //     center.x = middle_sgm[i][0];
  //     center.y = middle_sgm[i][1];
  //     circle( eroded, center, radius, Scalar( 0, 0, 255), FILLED, LINE_8);
  //     if ( i>=1)
  //     {
  //       Point prev_center;
  //       prev_center.x = middle_sgm[i-1][0];
  //       prev_center.y = middle_sgm[i-1][1];
  //       // cout << prev_center << endl;
  //       // line(eroded, center, prev_center, Scalar( 0, 255, 0), 2, LINE_8);
  //     }
  // }




  // vector<Vec2i> sortedList = sortListOnDistance(middle_sgm);

  // vector<Vec2i> middlePointList = middleSegments(sortedList, ropeThick);

  // for (size_t i=0; i<sortedList.size()-1; i++) {
  //   // cout << "Sorted List: \n" << sortedList[i] << endl;
  //   Point p1(sortedList[i][0],sortedList[i][1]), p2(sortedList[i+1][0],sortedList[i+1][1]);
  //   line(eroded, p1, p2, Scalar( 0, 255, 0), 2, LINE_8);
  // }
  // imshow("Dots",eroded);
  // imwrite("Dots.jpg", eroded);



  // Mat after_kernel,after_conv;
  // int kernel_data[9] = {-1, -1, -1, -1, 1, -1, -1, -1, -1};
  // int kernel_data[9] = {1,1,1,1,1,1,1,1,1};
  // Mat kernelH(3,3,CV_32F,kernel_data);
  // filter2D(gray_col,after_kernel, CV_32F,kernelH);
  // imshow("After kernel",after_kernel);
  // cout << otsu << endl;
  // matchTemplate(otsu, kernelH, after_conv, TM_CCORR_NORMED);
  // imshow("After template matching",after_kernel);

  // threshold(gray_col,tozero, 0, 255, THRESH_TOZERO_INV | THRESH_OTSU);
  // imshow("Foreground",tozero);

  // imshow("Final",otsu+tozero);


  // morphology(src);
  // histogramColorDetection(src);
  // cannyEdgeDetector(src);
  waitKey();
  return EXIT_SUCCESS;
}