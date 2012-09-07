#pragma once


#include <pcl/point_types.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include <string>
#include <vector>

void DrawCurvesSegmenter_onMouse( int event, int x, int y, int flags, void* voidThis);

class DrawCurvesSegmenter {
public:
  std::string m_windowName;
  std::vector<cv::Point2i> m_inside;
  std::vector<cv::Point2i> m_outside;
  enum State {
    BEFORE_START,
    BEFORE_INSIDE,
    DRAWING_INSIDE,
    BEFORE_OUTSIDE,
    DRAWING_OUTSIDE,
    DONE_DRAWING
  };
  State m_state;
  cv::Mat m_input;
  cv::Mat m_plotImg;

  DrawCurvesSegmenter();
  cv::Mat segment(cv::Mat in);
  void updatePlot();
};


void DrawRectSegmenter_onMouse( int event, int x, int y, int flags, void* voidThis);

class DrawRectSegmenter {
public:
  std::string m_windowName;
  cv::Point2i m_corner0;
  cv::Point2i m_corner1;

  enum State {
    BEFORE_START,
    BEFORE_RECT,
    DRAWING_RECT,
    DONE_DRAWING
  };
  State m_state;
  cv::Mat m_input;
  cv::Mat m_plotImg;

  DrawRectSegmenter();
  cv::Mat segment(cv::Mat in);
  void updatePlot();
};




//ColorCloudPtr maskCloud(ColorCloudPtr in, cv::Mat& mask, const sensor_msgs::CameraInfo& info);
