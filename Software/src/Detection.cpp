#include <ropeSegmentation.hpp>


namespace Detection
{

pair<double, double> calculateMinMaxValues(Mat image, char * title)
  {
    // ==========================================================
    /// @brief Method prints out the max and min values of a Mat object of the opencv Library
    /// @param image 
    /// @param title 
    /// @return minval, maxval
    // ==========================================================
    
    double minVal, maxVal;
    minMaxLoc(image, &minVal, &maxVal);
    cout << "-----------------------------------------" << endl;
    cout << "MAX " << title << ": " << maxVal << endl;
    cout << "MIN " << title << ": " << minVal << endl;
    cout << "-----------------------------------------" << endl;
    return make_pair(minVal, maxVal);
  }


void createDifferentKernels(int nbOfKernels, Mat kernel, vector<Mat> * kernels, vector<Mat> * rotationMatrices)
  {
    // ==========================================================
    /// @brief Functions that create a given number of rotated kernels starting from a given base kernel (in the shape of the rope)
    /// @param nbOfKernels amount of kernels over the span of 180°
    /// @param kernel the base kernel
    /// @param kernels the output kernels
    /// @param rotationMatrices the output rotation matrices that go together with the rotated kernels
    // ==========================================================

    // Initiation of the different 
    Mat kernelCpy, rotMatrix, frameRotated;
    double minAngle = -90.0;
    double maxAngle = 90.0;
    double deltaAngle = (maxAngle - minAngle)/((double) nbOfKernels);
    double currentAngle = minAngle;
    
    int diagonal = (int)sqrt((kernel.cols*kernel.cols)+(kernel.rows*kernel.rows));
    int newWidth = diagonal;
    int newHeight = diagonal;

    int offsetX = (newWidth - kernel.cols) / 2;
    int offsetY = (newHeight - kernel.rows) / 2;
    Mat targetMat = Mat::zeros(newWidth, newHeight, kernel.type());
    Point2f src_center(targetMat.cols/2.0F, targetMat.rows/2.0F); // center of the new square mat
    
    double minValVertical,maxValVertical;

    for(int i = 0; i < nbOfKernels; ++i)
    {
      
      kernel.copyTo(kernelCpy);
      currentAngle = minAngle + (((double) i) * deltaAngle);
      kernelCpy.copyTo(targetMat.rowRange(offsetY, offsetY + kernelCpy.rows).colRange(offsetX, offsetX + kernelCpy.cols));

      rotMatrix = getRotationMatrix2D(src_center, currentAngle, 1.0);
      warpAffine(targetMat, frameRotated, rotMatrix, Size(targetMat.cols,targetMat.rows),INTER_LINEAR, BORDER_CONSTANT, Scalar(0));

      frameRotated.copyTo(kernels->at(i));

      kernels->at(i)          = (kernels->at(i))/(sum(abs(kernels->at(i)))[0]);
      rotationMatrices->at(i) = rotMatrix;
    }
  }


double convolution(Mat image, Mat kernel)
  {
    // ==========================================================
    /// @brief this function returns the convolution value of the kernel on top of an equally large piece of an image
    /// @param image 
    /// @param kernel 
    /// @return a double value which is the result of the convolution 
    // ==========================================================
    
    // The image and kernel should have the same size
    Mat image_tmp = image;
    Mat kernel_tmp = kernel;
    Mat multiplication;

    multiplication = image_tmp.mul(kernel_tmp);
    double cocnvolutionValue = sum(multiplication)[0];

    return cocnvolutionValue;
  }


void tryDifferentKernelOrientations(int currentKernelIndex, int wantedKernelsToTry, Mat imagePart ,vector<Mat> * kernels, vector<double> * convolutionResults)
  {
    // ==========================================================
    /// @brief Function that tries different kernels on a the same piece of an image and returns the kernel with the highest convolution result
    /// @param currentKernelIndex the index of the kernel that came out as the best in the previous itteration
    /// @param wantedKernelsToTry the number of kernels left and right turned from the current kernel that are applied on the piece of image
    /// @param imagePart the image part on which the kernel is fit
    /// @param kernels a vector with all possible kernels
    /// @param convolutionResults a vector with all the outcomes of the convolutions, the ones that are not applied stay zero.
    // ==========================================================
    
    Mat imagePart_tmp;
    int nbKernels;
    int currentKernelIndexCpy = currentKernelIndex;
    for(int k=0; k<convolutionResults->size(); ++k)
    {
      convolutionResults->at(k) = 0.0;
    }
    

    if(wantedKernelsToTry > kernels->size()/2)
    {
      nbKernels = kernels->size()/2;
    }
    else 
    {
      nbKernels = wantedKernelsToTry;
    }

    for(int i = currentKernelIndexCpy - nbKernels; i<currentKernelIndexCpy + nbKernels+1; ++i)
    {
      int j;
      if(i < 0)
      {
        j = i + kernels->size();
      }
      else if(i >= kernels->size())
      {
        j = i - kernels->size();
      }
      else
      {
        j = i;
      }

      imagePart.copyTo(imagePart_tmp);
      convolutionResults->at(j) = convolution(imagePart_tmp,kernels->at(j));
    }
  }


int convolutionMap(Mat image,double maxValueAppearence, double minValueAppearence, double thresholdFactorOnMaxValue,  vector<Mat> * kernels, int currentKernelIndex, vector<int> * initialPoint)
  {
    // ==========================================================
    /// @brief funtion that performs the base convolution till it hits a certain threshold (when the rope is probably found)
    /// @param image the input image on which the rope has to be found
    /// @param maxValueAppearence the max pixel value of the input image
    /// @param minValueAppearence the min pixel value of the input image
    /// @param thresholdFactorOnMaxValue the factor that determines the threshold for when a good fit is found
    /// @param kernels the vector with all possible kernels
    /// @param currentKernelIndex the index of the base kernel
    /// @param initialPoint output point from where the tracking algorithm should start
    /// @return 
    // ==========================================================
    
    double maxConv = -255.0;
    double minConv = 255.0;
    double convolutionValue = 0.0;
    Mat imagePart;

    for(int i = kernels->at(currentKernelIndex).rows/2; i < image.rows - kernels->at(currentKernelIndex).rows/2; ++i)
    {
      for(int j = kernels->at(currentKernelIndex).cols/2; j < image.cols - kernels->at(currentKernelIndex).cols/2; ++j)
      {
        imagePart = image(Rect(j-kernels->at(currentKernelIndex).cols/2,i-kernels->at(currentKernelIndex).rows/2,kernels->at(currentKernelIndex).cols,kernels->at(currentKernelIndex).rows));
        convolutionValue = convolution(imagePart,kernels->at(currentKernelIndex));
        if(convolutionValue > maxConv){
          maxConv = convolutionValue;
        } 
        if(convolutionValue < minConv)
        {
          minConv = convolutionValue;
        }

        if(convolutionValue >= (maxValueAppearence - minValueAppearence)*thresholdFactorOnMaxValue + minValueAppearence)
        {
          initialPoint->at(0) = j;
          initialPoint->at(1) = i;

          return 1;
        }
      }
    }
    return 0;
  }


void drawCircles(Mat image, vector<vector<int>> * points)
  {
    // ==========================================================
    /// @brief draws a vector of circles on an image
    /// @param image 
    /// @param points 
    // ==========================================================
    Point point;
    for(int i=0; i<points->size(); ++i)
    {
      point.x = points->at(i).at(0);
      point.y = points->at(i).at(1);
      circle(image,point,2,Scalar(0,255,0),FILLED,LINE_AA);
    }
    imshow("Points on rope", image);
  }


void checkBestConvolutionResult(vector<double> * convolutionResults, double * bestResult, int * bestResultIndex)
  {
    // ==========================================================
    /// @brief out of a list of numbers gets the highest one with corresponding index
    /// @param convolutionResults 
    /// @param bestResult
    /// @param bestResultIndex
    // ==========================================================
    *bestResult = 0.0;
    for(int i = 0; i<convolutionResults->size(); ++i)
    {
      if(convolutionResults->at(i) > (*bestResult))
      {
        *bestResult = convolutionResults->at(i);
        *bestResultIndex = i;
      }
    }
  }


void ropeSegmentation( Mat image, vector<int> * initialPoint, int initialKernelIndex, vector<Mat> * kernels, vector<Mat> * rotationMatrices, vector<vector<int>> * segmentationPoints, int nbOfWantedKernelsToTry, Mat step)
  {
    // ==========================================================
    /// @brief method that follows the rope by taking a small step in the direction defined by the best fit of the previous point
    /// @param image 
    /// @param initialPoint 
    /// @param initialKernelIndex 
    /// @param kernels 
    /// @param rotationMatrices 
    /// @param segmentationPoints 
    /// @param nbOfWantedKernelsToTry 
    /// @param step 
    // ==========================================================
    
    double bestResult = 0.0;
    double convolutionValue = 0.0;
    double bestConvolutionResult = 1.0;
    Mat imagePart;
    int currentKernelIndex = initialKernelIndex;
    vector<int> currentPoint = {initialPoint->at(0),initialPoint->at(1)};
    vector<double> * convolutionResults = new vector<double>(kernels->size());
    Mat_<double> currentStep = step;
    Mat_<double> currentStep_tmp;
  

    while(bestConvolutionResult > 0.0)
    {  
      // Store point
      segmentationPoints->push_back(currentPoint);

      // Try different kernels
      imagePart = image(Rect((int)currentPoint[0]-kernels->at(currentKernelIndex).cols/2,
                            (int)currentPoint[1]-kernels->at(currentKernelIndex).rows/2,
                            kernels->at(currentKernelIndex).cols,
                            kernels->at(currentKernelIndex).rows));

      tryDifferentKernelOrientations(currentKernelIndex, nbOfWantedKernelsToTry, imagePart, kernels, convolutionResults);

      // Determine best kernel + rotation matrix
      checkBestConvolutionResult(convolutionResults, &bestResult, &currentKernelIndex);
      bestConvolutionResult = bestResult;

      // Take step forward
      currentStep_tmp = rotationMatrices->at(currentKernelIndex)(Rect(0,0,2,2)).t() * currentStep;
      currentStep(0,0) = currentStep_tmp(0,0);
      currentStep(1,0) = currentStep_tmp(1,0);


      currentPoint.at(0) = currentPoint.at(0) + round(currentStep.at<double>(0,0));
      currentPoint.at(1) = currentPoint.at(1) + round(currentStep.at<double>(1,0));

    }
  }
}