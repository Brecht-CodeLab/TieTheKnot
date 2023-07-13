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
// Perception structures

typedef struct rope_point_s{
   Vec2i point_coordinates;
   bool point_accessible;
}rope_point_t;

typedef struct rope_endpoints_s{
  rope_point_t rope_end1;
  rope_point_t rope_end2;
}rope_endpoints_t;

typedef struct rope_middlepoints_s{
  vector<rope_point_t> points;
  int nb_points;
}rope_middlepoints_t;

typedef struct gradient_segment_s{
  double gradient;
}gradient_segment_t;

typedef struct rope_segment_t{
  bool accessible;
  int z_level;
  rope_point_t begin_point;
  rope_point_t end_point;
  gradient_segment_t gradient;
}rope_segment_t;

typedef struct rope_segments_s{
  vector<rope_segment_t> segments;
  int nb_segments;
}rope_segments_t;

typedef struct perception_rope_s{
  rope_endpoints_t rope_endpoints;
  rope_middlepoints_t rope_middlepoints;
  rope_segments_t rope_segments;
}perception_rope_t;


typedef struct actuator_point_s{
   Vec2i point_coordinates;
   bool point_visible;
}actuator_point_t;

typedef struct robot_actuator_s{
  actuator_point_t base;
  actuator_point_t slider1;
  actuator_point_t slider2;
}robot_actuator_t;

typedef struct perception_actuators_s{
  robot_actuator_t actuator_1;
  robot_actuator_t actuator_2;
  bool actuators_visible;
}perception_actuators_t;

// Perception Activity

#endif