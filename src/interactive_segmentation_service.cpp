#include <ros/ros.h>
#include "interactive_segmentation.h"
#include "utils_pcl.h"
#include <snazzy_msgs/ProcessCloud.h>
#include <pcl/ros/conversions.h>
#include "cloud_ops.h"
#include <opencv2/highgui/highgui.hpp>
using namespace snazzy_msgs;

cv::Mat toCVMatImage2(const ColorCloudPtr cloud) {
  cv::Mat_<cv::Scalar_<uint8_t> > image(cloud->height, cloud->width);
  for (int i=0; i < cloud->height; ++i)
    for (int j=0; j < cloud->width; ++j) {
      ColorPoint& pt = cloud->at(j,i);
      image(i,j) = cv::Scalar_<uint8_t>(pt.b, pt.g, pt.r);
    }
  return image;
}

class SegServer {
public:
  DrawRectSegmenter m_drawRect;
  SegServer() {
  }
  bool callback(ProcessCloudRequest& req, ProcessCloudResponse& resp) {
    ColorCloudPtr cloud(new ColorCloud());
    pcl::fromROSMsg(req.cloud_in, *cloud);
    cv::Mat img = toCVMatImage(cloud);
    cv::Mat_<uint8_t> mask = m_drawRect.segment(img);
    ColorCloudPtr maskedCloud = maskCloud(cloud, mask);
//    maskedCloud = filterX(maskedCloud, -100, 100); // just to get rid of the nans
    maskedCloud = removeOutliers(maskedCloud, 1, 10);
    maskedCloud = clusterFilter(maskedCloud, .01, 50);
    pcl::toROSMsg(*maskedCloud, resp.cloud_out);
    return true;
  }
};

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "interactive_segmentation_service");
  ros::NodeHandle nh;
  SegServer server;
  ros::ServiceServer service = nh.advertiseService("interactive_segmentation", &SegServer::callback, &server);
  ros::spin();
}
