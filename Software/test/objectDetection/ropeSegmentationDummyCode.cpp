// }
  // Mat rot;
  // Rect2f bbox;
  // // Mat kernelCpy;
  // double minAngle = -90.0;
  // double maxAngle = 90.0;
  // double deltaAngle = (double)(maxAngle - minAngle)/((double) nbOfKernels);
  // double currentAngle = minAngle;

  // Point2f center((kernel.cols-1)/2.0, (kernel.rows-1)/2.0);
  

  // for(int i = 0; i < nbOfKernels; ++i)
  // {
  //   currentAngle = minAngle + (((double) i) * deltaAngle);
  //   rot = getRotationMatrix2D(center, currentAngle, 1.0);
  //   bbox = RotatedRect(Point2f(), kernel.size(), currentAngle).boundingRect2f();
  //   rot.at<double>(0,2) += bbox.width/2.0 - kernel.cols/2.0;
  //   rot.at<double>(1,2) += bbox.height/2.0 - kernel.rows/2.0;
  //   warpAffine(kernel,kernels->at(i),rot,bbox.size());
    
  //   // Make homogeneous and stor kernels and rotation matrices in the vectors
  //   kernels->at(i)          = (kernels->at(i))/(sum(abs(kernels->at(i)))[0]);
  //   rotationMatrices->at(i) = rot;


  //   // Check
  //   // imshow(to_string(i), kernels->at(i));
  //   // cout << "Current Angle: " << currentAngle << endl;
  //   // cout << "Kernel: " << kernels->at(i) << endl;
  //   // cout << "Rotatin matrix: " << rotationMatrices->at(i) << endl;

  // }




  // while(minValVertical < -1 || maxValVertical > 1)
    // {
    //   kernel.copyTo(kernelCpy);
    //   currentAngle = minAngle + (((double) i) * deltaAngle);
    //   kernelCpy.copyTo(targetMat.rowRange(offsetY, offsetY + kernelCpy.rows).colRange(offsetX, offsetX + kernelCpy.cols));
    //   rotMatrix = getRotationMatrix2D(src_center, currentAngle, 1.0);
    //   warpAffine(targetMat, frameRotated, rotMatrix, targetMat.size());
    //   tie(minValVertical,maxValVertical) = calculateMinMaxValues(frameRotated,"Values for rotated kernel");
    // }

    // calculateMinMaxValues(frameRotated,"Kernel i");











      // b = 0.5;
  // a = 1.0/(2.0*255.0);
  // Mat extractedRedImageTemp;
  // extractedRedImage.convertTo(extractedRedImageTemp,CV_32F,a,b);
  // imshow("RED EXTRACTED",extractedRedImageTemp);




// cout << "sum kernel" << sum(abs(kernelVerticalPieceRope))[0] << endl;
  // cout << "KERNEL BASE" << kernelVerticalPieceRope << endl;
  // Check kernel
  // double minValVerticalKernel,maxValVerticalKernel;
  // tie(minValVerticalKernel,maxValVerticalKernel) = calculateMinMaxValues(kernelVerticalPieceRope,"kernel");














// for(int i=0; i<rotatedKernels->size();++i){
  //   cout << "Kernel " << i << ": " << endl;
  //   cout << rotatedKernels->at(i).rows << "," << rotatedKernels->at(i).cols << endl; 
  // }
  
  // cout << rotatedKernels->at(0) << endl;






    // int x1 = offsetX;
    // int x2 = offsetX+kernelCpy.cols;
    // int x3 = offsetX;
    // int x4 = offsetX+kernelCpy.cols;

    // int y1 = offsetY;
    // int y2 = offsetY;
    // int y3 = offsetY+kernelCpy.rows;
    // int y4 = offsetY+kernelCpy.rows;
  // Rect boundRectangle(kernelCpy.cols,kernelCpy.rows,0,0);

    // Mat coordinate = (Mat_<double>(3,4) << x1, x2, x3, x4,
    //                                         y1, y2, y3, y4,
    //                                         1,  1,  1,  1 );
    // Mat rotCoordinate = rotMatrix * coordinate;

    // for(int i=0;i<4;i++){
    //    if(rotCoordinate.at<double>(0,i)<boundRectangle.x)
    //      boundRectangle.x=(int)rotCoordinate.at<double>(0,i); //access smallest 
    //    if(rotCoordinate.at<double>(1,i)<boundRectangle.y)
    //     boundRectangle.y=rotCoordinate.at<double>(1,i); //access smallest y
    //  }

    //  for(int i=0;i<4;i++){
    //    if(rotCoordinate.at<double>(0,i)>boundRectangle.width)
    //      boundRectangle.width=(int)rotCoordinate.at<double>(0,i); //access largest x
    //    if(rotCoordinate.at<double>(1,i)>boundRectangle.height)
    //     boundRectangle.height=rotCoordinate.at<double>(1,i); //access largest y
    //  }

    // boundRectangle.width=boundRectangle.width-boundRectangle.x;
    // boundRectangle.height=boundRectangle.height-boundRectangle.y;
