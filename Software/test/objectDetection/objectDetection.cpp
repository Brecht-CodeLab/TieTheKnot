#include <detection.hpp>


int main(int argc, char **argv)
{

  //===================================//
  // INITIALIZATION
  //===================================//
  // Import image
  Mat original, original_large,extractedR, extractedG, extractedB, fullImageHSV, tmpMask, redFiltered1, redFiltered2, greenFiltered, morphOpen, morphClose, eroded, dilated;
  CommandLineParser parser(argc, argv, "{@input | redRope2.jpg | input image}");
  
  /* Resize image
  TODO: make this function adaptable so that the size of the image is reduced:
  with the purpose of having lower computational cost, with the accuracy of the image still good enough
  for processing
  */

  int devider = 5;
  original = imread(samples::findFile(parser.get<String>("@input")), IMREAD_COLOR);
  resize(original, original, Size((int)original.size().width/devider, (int)original.size().height/devider), 1);
  // pyrDown( original, original, Size( original.cols/devider, original.rows/devider ));
  
  // -------------------------
  // Image error handle
  // -------------------------

  if (original.empty())
  {
    std::cout << "Could not open or find the image!\n"
              << std::endl;
    std::cout << "Usage: " << argv[0] << " <Input image>" << std::endl;
    return EXIT_FAILURE;
  }

  // Show and save image
  namedWindow("Initial Picture", WINDOW_AUTOSIZE);
  imshow("Initial Picture", original);
  // imwrite("InitialPicture.jpg", original);



  //===================================//
  // PREPROCESSING
  //===================================//
  // BGR to HSV
  cvtColor(original,fullImageHSV, COLOR_BGR2HSV);

  // -------------------------
  // Gaussian blur
  // -------------------------
  // Mat gaussian, gaussOriginal;
  // int gaussSize = 3;
  // GaussianBlur(fullImageHSV,gaussian, Size(gaussSize,gaussSize),0);
  // GaussianBlur(original,gaussOriginal, Size(gaussSize,gaussSize),0);
  // imshow("Gauss", gaussian);

  // -------------------------
  // Red filter
  // -------------------------
  // inRange(original, Scalar(0,40,45), Scalar(10, 255, 255), redFiltered1);
  // inRange(original, Scalar(160,40,45), Scalar(180, 255, 255), redFiltered2);
  // bitwise_or(redFiltered1, redFiltered2, redFiltered1);
  Mat bands[3],merged;
  split(original, bands);
  redFiltered1 = bands[2]; // RED
  // bitwise_and(redFiltered1,bands[2],redFiltered1);
  imshow("TMP Mask Red", redFiltered1);

  // -------------------------
  // Gray
  // -------------------------
  Mat hsv_channels[3];
  cv::split( original, hsv_channels );
  imshow("HSV to gray", hsv_channels[2]);
  // cvtColor(gaussian,gray_col, COLOR_HSV2GRAY);
  Mat gray_col = hsv_channels[2];
  imshow("Gray",gray_col);


  //===================================//
  // ROPE DETECTION
  //===================================//
  /*
  -------------------------
  Kernel Definition
  -------------------------
  Simple version of what it the kernel looks like:
  [[-1, 1, 1, -1] in vertical direction (but with larger dimensions)
  [-1, 1, 1, -1]
  [-1, 1, 1, -1]
  [-1, 1, 1, -1]  

  TODO: 
  */

  Mat ropeConv, ropeConvHor, ropeConvVer, ropeConvKernel, ropeConvKernelH;
  
  // First trials
  // Mat kernel_ver = (Mat_<double>(7,7) << -3, 1, 1, 2, 1, 1, -3, -3, 1, 1, 2, 1, 1, -3, -3, 1, 1, 2, 1, 1, -3, -3, 1, 1, 2, 1, 1, -3, -3, 1, 1, 2, 1, 1, -3, -3, 1, 1, 2, 1, 1, -3, -3, 1, 1, 2, 1, 1, -3);
  // Mat kernel_hor = (Mat_<double>(7,7) << -3, -3, -3, -3, -3, -3, -3, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, -3, -3, -3, -3, -3, -3, -3);
  // Mat kernel_ver = (Mat_<double>(7,7) << -3, 1, 1, 2, 1, 1, -3, -3, 1, 1, 2, 1, 1, -3, -3, 1, 1, 2, 1, 1, -3, -3, 1, 1, 2, 1, 1, -3, -3, 1, 1, 2, 1, 1, -3, -3, 1, 1, 2, 1, 1, -3, -3, 1, 1, 2, 1, 1, -3);
  // Mat kernel_hor = (Mat_<double>(7,7) << -3, -3, -3, -3, -3, -3, -3, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, -3, -3, -3, -3, -3, -3, -3);
  // Mat kernel_hor = (Mat_<double>(5,5) << -2, 1, 2, 1, -2, -2, 1, 2, 1, -2, -2, 1, 2, 1, -2, -2, 1, 2, 1, -2, -2, 1, 2, 1, -2);
  // Mat kernel_ver = (Mat_<double>(5,5) << -2, -2, -2, -2, -2, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, -2, -2, -2, -2, -2);
  // Mat kernel_hor = (Mat_<double>(3,3) << -1, -1, -1, 2, 2, 2, -1, -1, -1);
  // Mat kernel_ver = (Mat_<double>(3,3) << -1, 2, -1, -1, 2, -1, -1, 2, -1);

  // Actual kernel
  int kernel_size_v = 10;
  int kernel_size_h = 20;
  int diff_by_2 = abs(kernel_size_v - kernel_size_h)/2;

  Mat kernel_ver(kernel_size_h,kernel_size_h,CV_32F);
  Mat kernel_hor(kernel_size_h,kernel_size_h,CV_32F);  
  // Mat kernel_ver(kernel_size_h,kernel_size_h);
  // Mat kernel_hor(kernel_size_h,kernel_size_h); 
  vector<float> row = {-1, -1, -1, -1, -1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, -1, -1, -1, -1, -1};
  Mat zeros = Mat::zeros(1,kernel_size_h,CV_32S);
  cout << zeros << endl;
  for(int i = 0; i<kernel_size_h;++i){
    if(i>=diff_by_2 && i<diff_by_2+kernel_size_v){
      kernel_ver.row(i) = (Mat(row).t()) + 0;
    }
  }
  kernel_hor = kernel_ver.t();
 
  cout << kernel_ver << endl;
  cout << kernel_hor << endl;
  // imshow("kernel hor",kernel_hor);
  // imshow("kernel ver",kernel_ver);
  // imwrite("kernel_vert.png",kernel_ver);

  // Combine kernels horizontal and vertical
  Mat kernel = (1.0)*( kernel_ver);

  // Reduce kernel values
  // The kernel has to be reduced in value because there is saturation that occures:
  // TODO: The kernel values have to update automatically to not introduce magical numbers
  kernel = kernel;
  // imshow("kernel",kernel);
  cout << kernel << endl;



  // -------------------------
  // Convolution on the image
  // -------------------------
  // filter2D(gray_col, ropeConvVer, -1 , kernel_ver, Point(-1, -1), 0, 4);
  // filter2D(gray_col, ropeConvHor, -1 , kernel_hor, Point(-1, -1), 0, 4);
  // cout << "The first 10x10 pixels of the red filtered image" << redFiltered1(Rect(0,0,10,10)) << endl;
  // filter2D(redFiltered1, ropeConvKernel, -1 , kernel, Point(-1, -1), 0, 4);
  
  Mat gray_col32;
  gray_col.convertTo(gray_col32,CV_32F);

  double max_gray_32, min_gray_32;
  minMaxLoc(redFiltered1, &min_gray_32, &max_gray_32);

  cout << "max_val_red_32: " << max_gray_32 << endl;
  cout << "min_val_red_32: " << min_gray_32 << endl;

  filter2D(gray_col32, ropeConvKernel, -1, kernel, Point(-1, -1), 0, BORDER_CONSTANT);
  imshow("Rope Convoluted with Kernel", ropeConvKernel);
  // -------------------------
  // Saturation detection
  // -------------------------
  double maxval,minval;
  // minMaxLoc(ropeConvKernel,minval,maxval,minId,maxId);
  minMaxLoc(ropeConvKernel, &minval, &maxval);
  cout << "max: " <<maxval << endl;
  cout << "min: " <<minval << endl;
  // filter2D(gray_col, ropeConvKernelH, -1 , kernel_hor, Point(-1, -1), 0, 4);
  // ropeConvKernel = (ropeConvKernel + ropeConvKernelH)/2;
  // bitwise_or(ropeConvHor, ropeConvVer, ropeConv);  
  // imshow("Rope Convolution", ropeConvKernel);
  
  // Convolution to Binary
  Mat ropeBinary(ropeConvKernel.rows,ropeConvKernel.cols,CV_32F);
  // threshold(ropeConvKernel,ropeBinary, 0.8*maxval, maxval, THRESH_BINARY);
  // threshold(ropeConvKernel,ropeBinary, 200, 255, THRESH_BINARY);
  // imshow("Rope Binary",ropeBinary);
  
  ropeConvKernel = ropeConvKernel/(2*max(abs(minval),maxval));
  ropeConvKernel = ropeConvKernel + 1;

  minMaxLoc(ropeConvKernel, &minval, &maxval);
  cout << "max: " <<maxval << endl;
  cout << "min: " <<minval << endl;
  imshow("Rope Convolution after saturation handle", ropeConvKernel);


  // Color OR convolution
  Mat colorConv;
  // bitwise_and(ropeBinary,redFiltered1, colorConv);
  // imshow("Rope Red Conv", colorConv);

  


  // Foreground
  // Rect rectangle(0,0,original.cols,original.rows);
  // Mat frameForeground, frameForeground_a, frameForeground_b, foregroundResult, bgModel, fgModel;
  // grabCut(gaussian,frameForeground,rectangle, bgModel, fgModel,1,GC_INIT_WITH_RECT);
  // compare(frameForeground,cv::GC_BGD,frameForeground_a,cv::CMP_EQ);
  // compare(frameForeground,cv::GC_PR_BGD,frameForeground_b,cv::CMP_EQ);
  // foregroundResult = frameForeground_a + frameForeground_b;
  // Mat foreground(gaussian.size(),CV_8UC3,cv::Scalar(255,255,255));
  // gaussian.copyTo(foreground,foregroundResult);
  // imshow("Foreground", foreground);

  //=====================================//
  // Detection of the endpoints
  //=====================================//
  // Blue filter
  Mat blueFiltered;
  // inRange(fullImageHSV, Scalar(80,7,10), Scalar(140, 255, 255), blueFiltered);
  

  // // Magic numbers
  // int EROSION_TYPE = 0;
  // int EROSION_SIZE = 2;

  // int DILATION_TYPE = 0;
  // int DILATION_SIZE = 2;
  // // Erosion
  // Mat frameEroded;
  // Mat elementErosion = getStructuringElement(EROSION_TYPE,
  //                                     Size(2 * EROSION_SIZE + 1, 2 * EROSION_SIZE + 1),
  //                                     Point(EROSION_SIZE, EROSION_SIZE));
  // erode(blueFiltered, frameEroded, elementErosion);

  // // Dilation
  // Mat frameDilated;
  // Mat elementDilation = getStructuringElement(DILATION_TYPE,
  //                                     Size(2 * DILATION_SIZE + 1, 2 * DILATION_SIZE + 1),
  //                                     Point(DILATION_SIZE, DILATION_SIZE));
  // dilate(frameEroded, frameDilated, elementDilation);
  // imshow("Blue mask", frameDilated);


  
  waitKey(0);



  //=====================================//
  // Video
  //=====================================//
  // VideoCapture cap(0);
  // namedWindow("Video",WINDOW_AUTOSIZE );
  // if( ! cap.isOpened( ) ) {
	//   std::cerr << "couldn't open capture."<<std::endl;
	//   return -1;
  // }
  // Mat frame;

  // while( 1 ) {
  //   cap>>frame;
  //   if( frame.empty( ) ) 
  //     break;
	//   imshow ("Video", frame );
  //   waitKey(1);
	// }

  
  return 0;

}