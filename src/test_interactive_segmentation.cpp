#include "interactive_segmentation.h"
#include <opencv2/highgui/highgui.hpp>
int main(int argc, char* argv[]) {
  DrawRectSegmenter segmenter;
  cv::Mat flower = cv::imread("/tmp/plate.png");
  assert (!flower.empty());
  segmenter.segment(flower);
  cv::waitKey(0);
}
