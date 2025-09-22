#include "include/Matching3D.h"
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
#include <pcl/filters/extract_indices.h>
#include <pcl/common/common.h>


std::string model_filename_ = "data/milk.pcd";
std::string scene_filename_ = "data/Gmilk_cartoon_all_small_clorox.pcd";

//Algorithm params
bool use_hough_(false); // Usiamo Geometric Consistency
float model_ss_(0.01f);
float scene_ss_(0.03f);
float rf_rad_(0.015f);
float descr_rad_(0.02f);
float cg_size_(0.01f);
float cg_thresh_(5.0f);

void startMatching3D(Eigen::Matrix3f& transformation_matrix, Eigen::Vector3f& translation_vector)
{
	pcl::PointCloud<PointType>::Ptr      model(new pcl::PointCloud<PointType>());
	pcl::PointCloud<PointType>::Ptr      scene(new pcl::PointCloud<PointType>());

	pcl::PointCloud<PointType>::Ptr      model_keypoints(new pcl::PointCloud<PointType>());
	pcl::PointCloud<PointType>::Ptr      scene_keypoints(new pcl::PointCloud<PointType>());

	pcl::PointCloud<NormalType>::Ptr     model_normals(new pcl::PointCloud<NormalType>());
	pcl::PointCloud<NormalType>::Ptr     scene_normals(new pcl::PointCloud<NormalType>());

	pcl::PointCloud<DescriptorType>::Ptr model_descriptors(new pcl::PointCloud<DescriptorType>());
	pcl::PointCloud<DescriptorType>::Ptr scene_descriptors(new pcl::PointCloud<DescriptorType>());

	//
	//  Carico le point cloud
	//
	scene_filename_ = "data/Gabbiatrice.pcd";
	model_filename_ = "data/BordoLegatriceRandom.pcd";
	if (pcl::io::loadPCDFile(model_filename_, *model) < 0)
	{
		std::cout << "Error loading model cloud." << std::endl;
	}
	std::cout << "Model caricato; punti: " << model->size() << std::endl;
	if (pcl::io::loadPCDFile(scene_filename_, *scene) < 0)
	{
		std::cout << "Error loading scene cloud." << std::endl;
	}
	std::cout << "Scena caricata; punti: " << scene->size() << std::endl;

	//
	// Calcola la diagonale per scalare il raggio di campionamento
	//
	Eigen::Vector4f min_pt, max_pt;
	pcl::getMinMax3D(*model, min_pt, max_pt);
	float dx = max_pt.x() - min_pt.x();
	float dy = max_pt.y() - min_pt.y();
	float dz = max_pt.z() - min_pt.z();
	float model_diag = std::sqrt(dx * dx + dy * dy + dz * dz);

	//
	// Parametri dell'algoritmo basati sulla diagonale del model
	//
	model_ss_  = model_diag * 0.01f;
	scene_ss_  = model_diag * 0.015f;
	rf_rad_    = model_diag * 0.07f;
	descr_rad_ = model_diag * 0.15f;
	cg_size_   = model_diag * 0.07f;
	cg_thresh_ = 100;

	//
	//  Calcola le normali delle point cloud
	//
	pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
	norm_est.setKSearch(10);
	norm_est.setInputCloud(model);
	norm_est.compute(*model_normals);

	norm_est.setInputCloud(scene);
	norm_est.compute(*scene_normals);

	//
	//  Downsample sulle nuvole di punti per estrarre i keypoints
	//
	pcl::UniformSampling<PointType> uniform_sampling;
	uniform_sampling.setInputCloud(model);
	uniform_sampling.setRadiusSearch(model_ss_);
	uniform_sampling.filter(*model_keypoints);
	std::cout << "\nModel selected Keypoints: " << model_keypoints->size() << std::endl;

	uniform_sampling.setInputCloud(scene);
	uniform_sampling.setRadiusSearch(scene_ss_);
	uniform_sampling.filter(*scene_keypoints);
	std::cout << "Scene selected Keypoints: " << scene_keypoints->size() << std::endl;


	//
	//  Compute Descriptor for keypoints
	//
	pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType> descr_est;
	descr_est.setRadiusSearch(descr_rad_);

	descr_est.setInputCloud(model_keypoints);
	descr_est.setInputNormals(model_normals);
	descr_est.setSearchSurface(model);
	descr_est.compute(*model_descriptors);

	descr_est.setInputCloud(scene_keypoints);
	descr_est.setInputNormals(scene_normals);
	descr_est.setSearchSurface(scene);
	descr_est.compute(*scene_descriptors);

	//
	//  Find Model-Scene Correspondences with KdTree
	//
	pcl::CorrespondencesPtr model_scene_corrs(new pcl::Correspondences());

	pcl::KdTreeFLANN<DescriptorType> match_search;
	match_search.setInputCloud(model_descriptors);

	//  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
	for (std::size_t i = 0; i < scene_descriptors->size(); ++i)
	{
		std::vector<int> neigh_indices(1);
		std::vector<float> neigh_sqr_dists(1);
		if (!std::isfinite(scene_descriptors->at(i).descriptor[0])) //skipping NaNs
		{
			continue;
		}
		int found_neighs = match_search.nearestKSearch(scene_descriptors->at(i), 1, neigh_indices, neigh_sqr_dists);
		if (found_neighs == 1 && neigh_sqr_dists[0] < 0.20f) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
		{
			pcl::Correspondence corr(neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
			model_scene_corrs->push_back(corr);
		}
	}
	std::cout << "Correspondences found: " << model_scene_corrs->size() << std::endl;

	//
	//  Actual Clustering
	//
	std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
	std::vector<pcl::Correspondences> clustered_corrs;

	// Using GeometricConsistency Algorithm
	if (!use_hough_)
	{
		pcl::GeometricConsistencyGrouping<PointType, PointType> gc_clusterer;
		gc_clusterer.setGCSize(cg_size_);
		gc_clusterer.setGCThreshold(cg_thresh_);

		gc_clusterer.setInputCloud(model_keypoints);
		gc_clusterer.setSceneCloud(scene_keypoints);
		gc_clusterer.setModelSceneCorrespondences(model_scene_corrs);

		//gc_clusterer.cluster (clustered_corrs);
		gc_clusterer.recognize(rototranslations, clustered_corrs);
	}

	//
	//  Output results
	//
	std::cout << "Model instances found: " << rototranslations.size() << std::endl;
	for (std::size_t i = 0; i < rototranslations.size(); ++i)
	{
		std::cout << "\n    Instance " << i + 1 << ":" << std::endl;
		std::cout << "        Correspondences belonging to this instance: " << clustered_corrs[i].size() << std::endl;

		// Print the rotation matrix and translation vector
		transformation_matrix = rototranslations[i].block<3, 3>(0, 0);
		translation_vector    = rototranslations[i].block<3, 1>(0, 3);
	}

	//
	//  Visualization
	//
	pcl::visualization::PCLVisualizer viewer("Correspondence Grouping");
	viewer.addPointCloud(scene, "scene_cloud");

	pcl::PointCloud<PointType>::Ptr off_scene_model(new pcl::PointCloud<PointType>());
	pcl::PointCloud<PointType>::Ptr off_scene_model_keypoints(new pcl::PointCloud<PointType>());

	for (std::size_t i = 0; i < rototranslations.size(); ++i)
	{
		pcl::PointCloud<PointType>::Ptr rotated_model(new pcl::PointCloud<PointType>());
		pcl::transformPointCloud(*model, *rotated_model, rototranslations[i]);

		std::stringstream ss_cloud;
		ss_cloud << "instance" << i;

		pcl::visualization::PointCloudColorHandlerCustom<PointType> rotated_model_color_handler(rotated_model, 255, 0, 0);
		viewer.addPointCloud(rotated_model, rotated_model_color_handler, ss_cloud.str());
	}

	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}
}