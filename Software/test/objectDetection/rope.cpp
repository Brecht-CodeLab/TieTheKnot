#include <detection.hpp>


int main(int argc, char **argv)
{

  //===================================//
  // INITIALIZATION
  //===================================//
  // Import image
  Mat original, original_large;
  CommandLineParser parser(argc, argv, "{@input | redRope4.jpg | input image}");
  
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

  imshow("Initial Picture", original);
  
  double maxVal, minVal;

  Mat bands[3];
  split(original, bands);

  bands[0].convertTo(bands[0],CV_32F);
  bands[1].convertTo(bands[1],CV_32F);
  bands[2].convertTo(bands[2],CV_32F);
  Mat newRedImage = Mat((bands[2] - bands[0] - bands[1])/3.0);
  minMaxLoc(newRedImage, &minVal, &maxVal);
  cout << "max new Red Image: " << maxVal << endl;
  cout << "max new Red Image: " << minVal << endl;
  // newRedImage.convertTo(newRedImage,CV_32F,1.0,2.0*255.0/3.0);
  double a, b;
  b = (255.0*minVal)/(minVal-maxVal);
  a = (-255.0)/(minVal-maxVal);
  newRedImage.convertTo(newRedImage,CV_32F,a,b);
  imshow("New Red Image",newRedImage/255.0);
  minMaxLoc(newRedImage, &minVal, &maxVal);
  cout << "max new Red Image after conversion: " << maxVal << endl;
  cout << "max new Red Image after conversion: " << minVal << endl;

  Mat red32F;
  newRedImage.convertTo(red32F,CV_32F,2.0/255.0,-1.0);
  minMaxLoc(red32F, &minVal, &maxVal);
  cout << "max Red32F Image: " << maxVal << endl;
  cout << "max Red32F Image: " << minVal << endl;
  cout << red32F.type() << endl;
  
  // Mat red32FRescale;
  // red32F.convertTo(red32F,CV_32F,2.0/255.0,-1.0);
  
  // Actual kernel
  int kernel_size_v = 10;
  int kernel_size_h = 20;
  int diff_by_2 = abs(kernel_size_v - kernel_size_h)/2;

  Mat kernel_ver = Mat::zeros(kernel_size_h,kernel_size_h,CV_32F);
  Mat kernel_hor = Mat::zeros(kernel_size_h,kernel_size_h,CV_32F);  
  vector<float> row = {-1, -1, -1, -1, -1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, -1, -1, -1, -1, -1};
  // Mat kernel_ver = Mat::ones(kernel_size_h,kernel_size_h,CV_32F)*(-0.5);  
  // Mat kernel_hor = Mat::ones(kernel_size_h,kernel_size_h,CV_32F)*(-0.5);  
  // vector<float> row = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
  // Mat zeros = Mat::zeros(1,kernel_size_h,CV_32F);
  // cout << zeros << endl;
  // cout << zeros << endl;
  for(int i = 0; i<kernel_size_h;++i){
    if(i>=diff_by_2 && i<diff_by_2+kernel_size_v){
      kernel_ver.row(i) = (Mat(row).t()) + 0.0;
    }
  }
  kernel_hor = kernel_ver.t();
 
  // cout << kernel_ver << endl;
  // cout << kernel_hor << endl;
  // Mat kernel = (1.0)*( kernel_hor + kernel_ver)/(kernel_size_h*kernel_size_v);
  Mat kernel_h = (1.0)*( kernel_hor)/(kernel_size_h*kernel_size_v);
  Mat kernel_v = (1.0)*( kernel_ver)/(kernel_size_h*kernel_size_v);
  // cout << kernel << endl;
  Mat rope32Conv, rope32Conv_H,rope32Conv_V;
  filter2D(red32F, rope32Conv_H, -1, kernel_h, Point(-1, -1), 0, BORDER_ISOLATED);
  filter2D(red32F, rope32Conv_V, -1, kernel_v, Point(-1, -1), 0, BORDER_ISOLATED);
  // rope32Conv = (rope32Conv_V  )/1.0;
 

  cout << "Red32 Convolution: " << endl;
  minMaxLoc(rope32Conv_V, &minVal, &maxVal);
  cout << "max Red32 Convolution Image:" << maxVal << endl;
  cout << "max Red32 Convolution Image:" << minVal << endl;
  imshow("Red32F after Convolution",rope32Conv_V);



  Mat result,resultV,resultH;
  rope32Conv_V.convertTo(resultV,CV_32F,1.0,1.0);
  rope32Conv_H.convertTo(resultH,CV_32F,1.0,1.0);
  resultV.convertTo(resultV,CV_32F,1.0/2.0,0.0);
  resultH.convertTo(resultH,CV_32F,1.0/2.0,0.0);
  imshow("Result",resultV);

  double minValH, maxValH,minValV, maxValV;
  cout << "Red32 Convolution Result: " << endl;
  minMaxLoc(resultH, &minValH, &maxValH);
  minMaxLoc(resultV, &minValV, &maxValV);
  cout << "max Red32 Convolution Image Result:" << maxValV << endl;
  cout << "min Red32 Convolution Image Result:" << minValV << endl;



Mat ropeBinaryV,ropeBinaryH,ropeBinary;
cout << "Binary threshold V: " << (maxValV-minValV)*0.8 + minValV << endl;
threshold(resultH,ropeBinaryH,(maxValH-minValH)*0.8 + minValH , 1, THRESH_BINARY);
threshold(resultV,ropeBinaryV,(maxValV-minValV)*0.8 + minValV , 1, THRESH_BINARY);
bitwise_or(ropeBinaryH,ropeBinaryV,ropeBinary);
imshow("Rope Binary",ropeBinary);

waitKey(0);

}