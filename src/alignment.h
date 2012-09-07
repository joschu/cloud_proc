// mostly ripped off from http://pointclouds.org/documentation/tutorials/template_alignment.php

#pragma once
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>

class FeatureCloud {
public:
  // A bit of shorthand
  typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
  typedef pcl::PointCloud<pcl::Normal> SurfaceNormals;
  typedef pcl::PointCloud<pcl::FPFHSignature33> LocalFeatures;
  typedef pcl::search::KdTree<pcl::PointXYZ> SearchMethod;

  FeatureCloud();
  ~FeatureCloud() {
  }

  // Process the given cloud
  void setInputCloud(PointCloud::Ptr xyz);

  // Load and process the cloud in the given PCD file
  void loadInputCloud(const std::string &pcd_file);

  PointCloud::Ptr getPointCloud() const {
    return (xyz_);
  }

  // Get a pointer to the cloud of 3D surface normals
  SurfaceNormals::Ptr getSurfaceNormals() const {
    return (normals_);
  }

  // Get a pointer to the cloud of feature descriptors
  LocalFeatures::Ptr getLocalFeatures() const {
    return (features_);
  }

protected:
  // Compute the surface normals and local features
  void
  processInput();
  // Compute the surface normals
  void computeSurfaceNormals();
  void computeLocalFeatures();

private:
  // Point cloud data
  PointCloud::Ptr xyz_;
  SurfaceNormals::Ptr normals_;
  LocalFeatures::Ptr features_;
  SearchMethod::Ptr search_method_xyz_;

  // Parameters
  float normal_radius_;
  float feature_radius_;
};

class TemplateAlignment {
public:
  // A struct for storing alignment results
  struct Result {
    float fitness_score;
    Eigen::Matrix4f final_transformation;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  TemplateAlignment();
  ~TemplateAlignment() {
  }

  // Set the given cloud as the target to which the templates will be aligned
  void setTargetCloud(FeatureCloud &target_cloud);

  // Add the given cloud to the list of template clouds
  void addTemplateCloud(FeatureCloud &template_cloud);

  // Align the given template cloud to the target specified by setTargetCloud ()
  void align(FeatureCloud &template_cloud, TemplateAlignment::Result &result);

  // Align all of template clouds set by addTemplateCloud to the target specified by setTargetCloud ()
  void alignAll(std::vector<TemplateAlignment::Result, Eigen::aligned_allocator<Result> > &results);

  // Align all of template clouds to the target cloud to find the one with best alignment score
  int findBestAlignment(TemplateAlignment::Result &result);

private:
  // A list of template clouds and the target to which they will be aligned
  std::vector<FeatureCloud> templates_;
  FeatureCloud target_;

  // The Sample Consensus Initial Alignment (SAC-IA) registration routine and its parameters
  pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia_;
  float min_sample_distance_;
  float max_correspondence_distance_;
  int nr_iterations_;
};
