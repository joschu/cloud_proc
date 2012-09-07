#include "alignment.h"
#include "../hdfutil/hdfutil.hpp"
#include <boost/numeric/ublas/matrix.hpp>
#include <Eigen/Dense>
#include "pcl_typedefs.h"
#include "cloud_ops.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
CloudPtr downsampleCloud1(const CloudPtr in, float sz) {
  pcl::PointCloud<Point>::Ptr out(new pcl::PointCloud<Point>());
  pcl::VoxelGrid<Point> vg;
  vg.setInputCloud(in);
  vg.setLeafSize(sz,sz,sz);
  vg.filter(*out);
  return out;
}


Eigen::MatrixXf toEigen(const ublas::matrix<float>& in) {
  Eigen::MatrixXf out(in.size1(), in.size2());
  for (int i=0; i < out.rows(); ++i)
    for (int j=0; j < out.cols(); ++j)
      out(i,j) = in(i,j);
  return out;
}

int main(int argc, char* argv[]) {
  H5::H5File mug0file("/home/joschu/Data/registration/mug0.seg.h5", H5F_ACC_RDONLY);
  H5::H5File mug1file("/home/joschu/Data/registration/mug1.seg.h5", H5F_ACC_RDONLY);
//  H5::H5File mug2file("/home/joschu/Data/registration/mug2.seg.h5", H5F_ACC_RDONLY);

  Eigen::MatrixXf xyz = toEigen(hdfutil::ReadMatrix<float>(mug0file, "mug/xyz"));
  printf("%i %i\n", xyz.rows(), xyz.cols());
  CloudPtr cloud0 = toPointCloud(xyz);

  xyz = toEigen(hdfutil::ReadMatrix<float>(mug1file, "mug/xyz"));
  CloudPtr cloud1 = toPointCloud(xyz);


  FeatureCloud template_cloud;
  template_cloud.setInputCloud(downsampleCloud1(cloud0, .01));


  // Assign to the target FeatureCloud
  FeatureCloud target_cloud;
  target_cloud.setInputCloud (downsampleCloud1(cloud1, .01));

  // Set the TemplateAlignment inputs
  TemplateAlignment template_align;
  template_align.setTargetCloud(target_cloud);
  TemplateAlignment::Result result;
  template_align.align(template_cloud, result);
  CloudPtr cloud0_tf(new Cloud());
  pcl::transformPointCloud(*cloud0, *cloud0_tf, result.final_transformation);


  pcl::visualization::PCLVisualizer vis;
  vis.addPointCloud(addColor(cloud0, 255, 0, 0), "0");
  vis.addPointCloud(addColor(cloud0_tf, 0, 255, 0), "1");
  vis.addPointCloud(addColor(cloud1, 0, 0, 255), "2");
  vis.spin();
}
