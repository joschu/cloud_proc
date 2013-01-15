#include "interactive_segmentation.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <string>
using namespace std;


cv::Mat_<uint8_t> gcToMask(cv::Mat_<uint8_t> in) {
  cv::Mat_<uint8_t> out = in.clone();
  for (int i=0; i < in.rows; ++i)
    for (int j=0; j < in.cols; ++j)
      out(i,j) %= 2;
  return out;
}

void gcPlotMask(cv::Mat_<uint8_t> mask, cv::Mat plotImg) {
  vector< vector<cv::Point2i> > contours;
  cv::findContours(mask, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);
  cv::drawContours(plotImg, contours, -1, cv::Scalar(0,255,0), 2);
}

void DrawCurvesSegmenter_onMouse( int event, int x, int y, int flags, void* voidThis) {
  DrawCurvesSegmenter* this0 = static_cast<DrawCurvesSegmenter*>(voidThis);

  if (this0->m_state == DrawCurvesSegmenter::BEFORE_INSIDE) {
    if (event == CV_EVENT_LBUTTONDOWN) {
      this0->m_inside.push_back(cv::Point2i(x,y));
      this0->m_state = DrawCurvesSegmenter::DRAWING_INSIDE;
    }
  }
  else if (this0->m_state == DrawCurvesSegmenter::DRAWING_INSIDE) {
    if (event == CV_EVENT_MOUSEMOVE) {
      this0->m_inside.push_back(cv::Point2i(x,y));
    }
    else if (event == CV_EVENT_LBUTTONUP) {
      this0->m_state = DrawCurvesSegmenter::BEFORE_OUTSIDE;
    }
  }
  else if (this0->m_state == DrawCurvesSegmenter::BEFORE_OUTSIDE) {
    if (event == CV_EVENT_LBUTTONDOWN) {
      this0->m_outside.push_back(cv::Point2i(x,y));
      this0->m_state = DrawCurvesSegmenter::DRAWING_OUTSIDE;
    }
  }
  else if (this0->m_state == DrawCurvesSegmenter::DRAWING_OUTSIDE) {
    if (event == CV_EVENT_MOUSEMOVE) {
      this0->m_outside.push_back(cv::Point2i(x,y));
    }
    else if (event == CV_EVENT_LBUTTONUP) {
      this0->m_state = DrawCurvesSegmenter::DONE_DRAWING;
    }
  }

}

DrawCurvesSegmenter::DrawCurvesSegmenter() {
  m_windowName = "draw two curves";
  m_state = BEFORE_START;
  cv::namedWindow(m_windowName, cv::WINDOW_NORMAL);
}

void DrawCurvesSegmenter::updatePlot() {
  m_input.copyTo(m_plotImg);
  int nInside = m_inside.size();
  const cv::Point2i* insideDataPtr = m_inside.data();
  cv::polylines(m_plotImg, &insideDataPtr, &nInside, 1, 0, cv::Scalar(255,0,0));
  int nOutside = m_outside.size();
  const cv::Point2i* outsideDataPtr = m_outside.data();
  cv::polylines(m_plotImg, &outsideDataPtr, &nOutside, 1, 0, cv::Scalar(0, 255,0));
}

cv::Mat DrawCurvesSegmenter::segment(cv::Mat in) {
  m_input = in;
  cv::setMouseCallback(m_windowName, &DrawCurvesSegmenter_onMouse, this);
  m_state = BEFORE_INSIDE;
  while (m_state != DONE_DRAWING) {
    updatePlot();
    cv::imshow(m_windowName, m_plotImg);
    cv::waitKey(10);
  }


  vector<cv::Point2i> allPts = m_inside;
  for (int i=0; i < m_outside.size(); ++i) allPts.push_back(m_outside[i]);
  cv::Rect bb = cv::boundingRect(allPts);

  cv::Mat_<uint8_t> mask(m_input.rows, m_input.cols, (uint8_t)0);
  cv::Mat_<uint8_t> inMask(m_input.rows, m_input.cols, (uint8_t)0);
  cv::Mat_<uint8_t> outMask(m_input.rows, m_input.cols, (uint8_t)0);


  for (int i=0; i < m_inside.size(); ++i) inMask(m_inside[i]) = 1;
  for (int i=0; i < m_outside.size(); ++i) outMask(m_outside[i]) = 1;
  cv::Mat_<float> inDist;
  cv::Mat_<float> outDist;
  cv::distanceTransform(1-inMask, inDist, CV_DIST_C, 3);
  cv::distanceTransform(1-outMask, outDist, CV_DIST_C, 3);


  for (int i=0; i < mask.rows; ++i)
    for (int j=0; j < mask.cols; ++j)
      mask(i,j) = (inDist(i,j) < outDist(i,j)) ? cv::GC_PR_FGD : cv::GC_PR_BGD;


  for (int i=0; i < m_inside.size(); ++i) mask(m_inside[i]) = cv::GC_FGD;
  for (int i=0; i < m_outside.size(); ++i) mask(m_outside[i]) = cv::GC_BGD;

  cv::Mat bgdModel, fgdModel;
  cv::Rect rect;
  cv::grabCut( cv::Mat(m_input, bb), cv::Mat(mask, bb), rect, bgdModel, fgdModel, 1, cv::GC_INIT_WITH_MASK );

  for (int i=0; i < mask.rows; ++i)
    for (int j=0; j < mask.cols; ++j)
      mask(i,j) %= 2;


  vector< vector<cv::Point2i> > contours;
  cv::findContours(mask, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);
  cv::drawContours(m_plotImg, contours, -1, cv::Scalar(0,255,0), 2);
  cv::imshow(m_windowName,m_plotImg);
  cv::waitKey(10);


  cv::setMouseCallback(m_windowName, NULL, NULL);

  return mask;
}



void DrawRectSegmenter_onMouse( int event, int x, int y, int flags, void* voidThis) {
  DrawRectSegmenter* this0 = static_cast<DrawRectSegmenter*>(voidThis);

  if (this0->m_state == DrawRectSegmenter::BEFORE_RECT) {
    if (event == CV_EVENT_LBUTTONDOWN) {
      this0->m_corner0 = cv::Point2i(x,y);
      this0->m_corner1 = cv::Point2i(x,y);
      this0->m_state = DrawRectSegmenter::DRAWING_RECT;
    }
  }
  else if (this0->m_state == DrawRectSegmenter::DRAWING_RECT) {
    if (event == CV_EVENT_MOUSEMOVE) {
      this0->m_corner1 = cv::Point2i(x,y);
    }
    else if (event == CV_EVENT_LBUTTONUP) {
      this0->m_state = DrawRectSegmenter::DONE_DRAWING;
    }
  }
  else if (this0->m_state == DrawRectSegmenter::DONE_DRAWING) {
    return;
  }
}



DrawRectSegmenter::DrawRectSegmenter() {
  m_windowName = "draw rectangle";
  m_state = BEFORE_START;
  cv::namedWindow(m_windowName, cv::WINDOW_NORMAL);
}

void DrawRectSegmenter::updatePlot() {
  m_input.copyTo(m_plotImg);
  if (m_state >= DRAWING_RECT) {
    cv::rectangle(m_plotImg, m_corner0, m_corner1, cv::Scalar(255,0,0), 2);
    cv::rectangle(m_plotImg, m_corner0*.25 + m_corner1*.75, m_corner0*.75 + m_corner1*.25, cv::Scalar(0,0,255), 2);
  }
}

cv::Mat DrawRectSegmenter::segment(cv::Mat in) {
  m_input = in;
  cv::setMouseCallback(m_windowName, DrawRectSegmenter_onMouse, this);
  m_state = BEFORE_RECT;
  while (m_state != DONE_DRAWING) {
    updatePlot();
    cv::imshow(m_windowName, m_plotImg);
    cv::waitKey(10);
  }


  cv::Mat_<uint8_t> mask(m_input.rows, m_input.cols, (uint8_t)0);


  vector<cv::Point2i> m_corners;
  m_corners.push_back(m_corner0);
  m_corners.push_back(m_corner1);
  cv::Rect bb = cv::boundingRect(m_corners);
  cv::Rect inside = cv::Rect(bb.x + bb.width/4, bb.y + bb.height/4, bb.width/2, bb.height/2);
  cv::Mat(mask, bb) = cv::GC_PR_BGD;
  cv::Mat(mask, inside) = cv::GC_PR_FGD;

  cv::Mat bgdModel(1,65, CV_64FC1, (double)0), fgdModel(1,65, CV_64FC1,(double)0);
  bgdModel.resize(1, 65);
  fgdModel.resize(1, 65);
  cv::Rect rect;

  long int start = cv::getTickCount();
  cv::grabCut( cv::Mat(m_input, bb), cv::Mat(mask, bb), rect, bgdModel, fgdModel, 5, cv::GC_INIT_WITH_MASK );
  printf("time: %.3f\n", (cv::getTickCount()-start)/cv::getTickFrequency());



  mask = gcToMask(mask);
  gcPlotMask(mask, m_plotImg);

  cv::imshow(m_windowName,m_plotImg);
  cv::waitKey(10);

  cv::setMouseCallback(m_windowName, NULL, NULL);

  return mask;
}



