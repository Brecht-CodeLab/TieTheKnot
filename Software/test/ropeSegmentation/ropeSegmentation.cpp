#include <ropeSegmentation.hpp>




int imageErrorHandle(Mat image, char**argv)
{
  if (image.empty())
  {
    std::cout << "Could not open or find the image!\n" << std::endl;
    std::cout << "Usage: " << argv[0] << " <Input image>" << std::endl;
    return EXIT_FAILURE;
  }
}

int main(int argc, char **argv)
{
  /*
  - TODO: make this function adaptable so that the size of the image is reduced:
  with the purpose of having lower computational cost, with the accuracy of the image still good enough
  for processing
  - TODO: 
  */

  //=================================== 
  // DETECTION NAMESPACE
  //===================================
  // using namespace Detection;

  //===================================
  // INITIALIZATION
  //===================================
  Mat original;
  CommandLineParser parser(argc, argv, "{@input | ../images/input/redRope4.jpg | input image}");
  original = imread(samples::findFile(parser.get<String>("@input")), IMREAD_COLOR);
  imageErrorHandle(original,argv);



  //===================================
  // PYRAMID RESIZING
  //===================================
  int devider = 2;
  // resize(original, original, Size((int)original.size().width/devider, (int)original.size().height/devider), 1);
  pyrDown( original, original, Size( original.cols/devider, original.rows/devider ));
  pyrDown( original, original, Size( original.cols/devider, original.rows/devider ));
  imshow("Resized Picture", original);


  //===================================
  // CHANNEL SPLITTING
  //===================================
  Mat channels[3];
  split(original, channels); 

  channels[0].convertTo(channels[0],CV_32F);
  channels[1].convertTo(channels[1],CV_32F);
  channels[2].convertTo(channels[2],CV_32F);



  //===================================
  // EXTRACT RED COLOR - REMOVE OTHERS
  //===================================
  // The image is now a 2x2x1 matrix with values between -2*255/3 and 255/3
  Mat extractedRedImage = (Mat_<double>((channels[2] - channels[0] - channels[1])/3.0));
  
  // Shifting the values back to a -255 and 255 band (min - max) gives:
  double minValVertical,maxValVertical;
  double a, b;
  tie(minValVertical,maxValVertical) = calculateMinMaxValues(extractedRedImage,"red filtered image");
  b = 255.0*(minValVertical+maxValVertical)/(minValVertical-maxValVertical);
  a = -2.0*255.0/(minValVertical-maxValVertical); 
  extractedRedImage.convertTo(extractedRedImage,CV_32F,a,b);


  //===================================
  // KERNEL DEFINITIONS
  //===================================

  vector<float> row = { -1, -1, -1, -1, 1, 1, 1, 1, 1, 1, 1, 1, -1, -1, -1, -1};
  int kernelSizeH = row.size();
  int kernelSizeV = 10; 

  // Kernels initiation
  Mat kernelVerticalPieceRope = Mat::zeros(kernelSizeV,kernelSizeH,CV_32F);
  
  // Fill kernels with values
  for(int i = 0; i<kernelSizeV;++i){
    kernelVerticalPieceRope.row(i) = (Mat(row).t()) + 0.0;
  }
  kernelVerticalPieceRope = kernelVerticalPieceRope.t();
  
  // Create rotated set of kernels
  int numberOfKernels = 40;
  vector<Mat> * rotatedKernels = new vector<Mat>(numberOfKernels);
  vector<Mat> * rotationMatrices = new vector<Mat>(numberOfKernels);
  createDifferentKernels(numberOfKernels,kernelVerticalPieceRope,rotatedKernels, rotationMatrices);


  //===================================
  // CONVOLUTION
  //===================================
  // Init parameters and variables
  double thresholdFactor = 0.7;
  vector<int> * initialPoint = new vector<int>(2,0);
  int initialKernel = numberOfKernels/2;
  int nbOfWantedKernelsToTry = (30 * numberOfKernels)/180 + 1; // The number of the 

  vector<vector<int>> * segmentationPoints = new vector<vector<int>>; // The points that on the line
 
  // Step definition
  Mat_<double> step {5.0, 0.0};
  



  // Calculate min and max values of the red image
  tie(minValVertical,maxValVertical) = calculateMinMaxValues(extractedRedImage,"red image rescaled to -255 -- 255");

  // Convolution over image to find best fit for first point of follwing the line
  convolutionMap(extractedRedImage,maxValVertical,minValVertical,thresholdFactor,rotatedKernels,initialKernel,initialPoint);

  //===================================
  // SEGMENTATION
  //===================================

  // Rope Segmentation
  ropeSegmentation(extractedRedImage, initialPoint, initialKernel, rotatedKernels, rotationMatrices, segmentationPoints, nbOfWantedKernelsToTry, step);

  
  // Plot points
  Mat originalCpy;
  original.copyTo(originalCpy);
  drawCircles(originalCpy,segmentationPoints);
 
  waitKey(0);


}

