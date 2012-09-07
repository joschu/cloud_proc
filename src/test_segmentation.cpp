#include "segmentation.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include "utils_pcl.h"
#include <sstream>
using namespace std;

int main(int argc, char* argv[]) {
  ColorCloudPtr cloud = readPCD("/home/joschu/Data/scp/three_objs.pcd");
  vector<ColorCloudPtr> segments = segmentPointCloud(cloud);
  printf("%i segments found\n", segments.size());
  pcl::visualization::PCLVisualizer vis;

  for (int i=0; i < segments.size(); ++i) {
    printf("segment %i has size %i\n", i, segments[i]->size());
    stringstream name;
    name << i;
    vis.addPointCloud(segments[i], name.str());
  }
  vis.spin();
}
