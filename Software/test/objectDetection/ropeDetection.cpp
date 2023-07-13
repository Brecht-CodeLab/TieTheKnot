#include <detection.hpp>

int imageErrorHandle(Mat image, char**argv)
{
  if (image.empty())
  {
    std::cout << "Could not open or find the image!\n" << std::endl;
    std::cout << "Usage: " << argv[0] << " <Input image>" << std::endl;
    return EXIT_FAILURE;
  }
}

double calculateMinMaxValues(Mat image,char * title)
{
  double minVal, maxVal;
  minMaxLoc(image, &minVal, &maxVal);
  cout << "-----------------------------------------" << endl;
  cout << "MAX " << title << ": " << maxVal << endl;
  cout << "MIN " << title << ": " << minVal << endl;
  cout << "-----------------------------------------" << endl;
  return minVal,maxVal;
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
  Mat extractedRedImage = Mat((channels[2] - channels[0] - channels[1])/3.0);
  
  // Shifting the values back to a -255 and 255 band gives:
  double a, b;
  b = 255.0/3.0;
  a = 2.0;
  extractedRedImage.convertTo(extractedRedImage,CV_32F,a,b);

  // Test if the values are indeed between -255 and 255
  calculateMinMaxValues(extractedRedImage,"Extracted Red Image");


  //===================================
  // KERNEL DEFINITIONS
  //===================================
  // In this definition, we start from a realy easy kernel that is build up from ones and minus ones. Of cours this kernel can be improved, this is dicussed later in this document.
  // The kernel will look like this:
  // Vertical piece of rope detection: [-1 .. -1 | 1 ... 1 | -1 .. -1] with the ones the size of the rope. In this case we take 10 pixels. The reasoning for this width is given in the markdown document.
  // Horizontal piece of rope detection: transpose(vertical)
  // For the combination of both observations, we use 2 different reasonings:
  
  // 1) Combine both kernels to one kernel that looks something lik:
  // [ 0 | -1 |  0]
  // [-1 |  2 | -1]
  // [ 0 | -1 |  0]
  // Although this now doesn't look like we are looking for a streight piece of rope but just a square of the size of the rope, this method puts out some oke results
  
  // 2) The second method serves as a possible answer to the intuition problem with the first option. We now split the the detection of vertical and horizontal pieces of the rope in two and combine both results afterwards. A problem with this is that the corners (nor completely vertical or horizontal) will nog be as good detected as with the other option.
  
  // Input
  /*
  @var kernelSizeV: This is the amount of rows and is again based on the fact that we want to detect pieces of  around 10 px long (not wide).
  */
  vector<float> row = {-1, -1, -1, -1, -1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, -1, -1, -1, -1, -1};
  int kernelSizeH = row.size();
  int kernelSizeV = kernelSizeH/2; 

  // Kernels initiation
  Mat kernelVerticalPieceRope = Mat::zeros(kernelSizeH,kernelSizeH,CV_32F);
  Mat kernelHorizontalPieceRope = Mat::zeros(kernelSizeH,kernelSizeH,CV_32F);  

  // Fill kernels with values
  int differenceSize = abs(kernelSizeV - kernelSizeH)/2;
   for(int i = 0; i<kernelSizeH;++i){
    if(i>=differenceSize && i<differenceSize+kernelSizeV){
      kernelVerticalPieceRope.row(i) = (Mat(row).t()) + 0.0;
    }
  }
  kernelHorizontalPieceRope = kernelVerticalPieceRope.t();

  // Make kernels homogeneous
  kernelHorizontalPieceRope = (kernelHorizontalPieceRope)/(sum(abs(kernelHorizontalPieceRope))[0]);
  kernelVerticalPieceRope = (kernelVerticalPieceRope)/(sum(abs(kernelVerticalPieceRope))[0]);

  //===================================
  // METHOD 1
  //===================================
  Mat kernelCombined = kernelHorizontalPieceRope + kernelVerticalPieceRope;
  Mat convolutedImageCombinedPieceRope;
  filter2D(extractedRedImage, convolutedImageCombinedPieceRope, -1, kernelCombined, Point(-1, -1), 0, BORDER_ISOLATED);

  // Rescale the image value to [0 , 1]
  a = 1.0/(2.0*255.0);
  b = 1.0/2.0;
  convolutedImageCombinedPieceRope.convertTo(convolutedImageCombinedPieceRope,CV_32F,a,b);

  // Check image values
  double minValCombined, maxValCombined;
  minValCombined, maxValCombined = calculateMinMaxValues(convolutedImageCombinedPieceRope,"rescaled to 0-1 after convolution image Combined");

  // Binary
  Mat binaryCombinedPieceRope;
  threshold(convolutedImageCombinedPieceRope,binaryCombinedPieceRope,(maxValCombined-minValCombined)*0.91 + minValCombined , 1, THRESH_BINARY);
  
  // Check binaries
  calculateMinMaxValues(binaryCombinedPieceRope,"Binary vertical should be 0 or 1");
  imshow("Binary Image Combined",binaryCombinedPieceRope);






  //===================================
  // METHOD 2
  //===================================
  Mat convolutedImageHorizontalPieceRope, convolutedImageVerticalPieceRope;
  filter2D(extractedRedImage, convolutedImageHorizontalPieceRope, -1, kernelHorizontalPieceRope, Point(-1, -1), 0, BORDER_ISOLATED);
  filter2D(extractedRedImage, convolutedImageVerticalPieceRope, -1, kernelVerticalPieceRope, Point(-1, -1), 0, BORDER_ISOLATED);
  
  // Check the outcome
  calculateMinMaxValues(convolutedImageVerticalPieceRope,"convoluted image vertical");

  // Rescale the image value to [0 , 1]
  a = 1.0/(2.0*255.0);
  b = 1.0/2.0;
  convolutedImageVerticalPieceRope.convertTo(convolutedImageVerticalPieceRope,CV_32F,a,b);
  convolutedImageHorizontalPieceRope.convertTo(convolutedImageHorizontalPieceRope,CV_32F,a,b);

  // Check min max values
  double minValHorizontal,maxValHorizontal,minValVertical,maxValVertical;
  minValVertical, maxValVertical = calculateMinMaxValues(convolutedImageVerticalPieceRope,"rescaled to 0-1 after convolution image verctical");
  minValHorizontal, maxValHorizontal = calculateMinMaxValues(convolutedImageHorizontalPieceRope,"rescaled to 0-1 after convolution image horizontal");
  // imshow("Convoluted Image Vertical",convolutedImageVerticalPieceRope);


  // Binary
  Mat binaryHorizontalPieceRope, binaryVerticalPieceRope;
  threshold(convolutedImageVerticalPieceRope,binaryVerticalPieceRope,(maxValVertical-minValVertical)*0.91 + minValVertical , 1, THRESH_BINARY);
  threshold(convolutedImageHorizontalPieceRope,binaryHorizontalPieceRope,(maxValHorizontal-minValHorizontal)*0.91 + minValHorizontal , 1, THRESH_BINARY);

  // Check binaries
  calculateMinMaxValues(binaryVerticalPieceRope,"Binary vertical should be 0 or 1");
  // imshow("Binary Image Vertical",binaryVerticalPieceRope);
  // imshow("Binary Image Horizontal",binaryHorizontalPieceRope);

  // Combine binaries
  Mat binaryImageCombined;
  bitwise_or(binaryVerticalPieceRope, binaryHorizontalPieceRope, binaryImageCombined);
  // imshow("Binary Image Combined",binaryImageCombined);


  waitKey(0);
}






