#pragma once

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>

typedef pcl::PointXYZ PointType;
typedef pcl::Normal NormalType;
typedef pcl::ReferenceFrame RFType;
typedef pcl::SHOT352 DescriptorType;

// Matching 3Dd debug con un file model e scene già impostati
void startMatching3DFromFile(Eigen::Matrix3f& transformation_matrix, Eigen::Vector3f& translation_vector, std::string model_filename_ = "data/milk.pcd", std::string scene_filename_ = "data/Gmilk_cartoon_all_small_clorox.pcd");


void startMatching3DFromCamera(Eigen::Matrix3f& transformation_matrix, Eigen::Vector3f& translation_vector, std::string model_filename_, pcl::PointCloud<PointType>::Ptr scene);