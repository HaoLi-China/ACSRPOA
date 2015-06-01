#include "detect_change.h"

//Euclidean Cluster Extraction
void ECE(PointCloudPtr_RGB_NORMAL cloud, std::vector<PointCloudPtr_RGB_NORMAL> &cluster_points){
  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<Point_RGB_NORMAL>::Ptr tree (new pcl::search::KdTree<Point_RGB_NORMAL>);
  tree->setInputCloud (cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<Point_RGB_NORMAL> ec;
  ec.setClusterTolerance (0.010); // 1cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (500000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud);
  ec.extract (cluster_indices);

  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    PointCloudPtr_RGB_NORMAL cloud_cluster (new PointCloud_RGB_NORMAL);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++){
      cloud_cluster->points.push_back (cloud->points[*pit]); //*
    }
    cluster_points.push_back(cloud_cluster);

    j++;
  }
}

//detect change of two point cloud
void detect_change(PointCloudPtr_RGB_NORMAL cloud0, PointCloudPtr_RGB_NORMAL cloud1, PointCloudPtr_RGB_NORMAL result0, PointCloudPtr_RGB_NORMAL result1){
  PointCloudPtr_RGB_NORMAL cloud_tem (new PointCloud_RGB_NORMAL);

  // Octree resolution - side length of octree voxels
  float resolution = 0.01f;

  // Instantiate octree-based point cloud change detection class
  pcl::octree::OctreePointCloudChangeDetector<Point_RGB_NORMAL> octree0 (resolution);

  // Add points from cloudA to octree
  octree0.setInputCloud (cloud0);
  octree0.addPointsFromInputCloud ();

  // Switch octree buffers: This resets octree but keeps previous tree structure in memory.
  octree0.switchBuffers ();

  // Add points from cloudB to octree
  octree0.setInputCloud (cloud1);
  octree0.addPointsFromInputCloud ();

  std::vector<int> newPointIdxVector0;

  // Get vector of point indices from octree voxels which did not exist in previous buffer
  octree0.getPointIndicesFromNewVoxels (newPointIdxVector0);

  for (size_t i = 0; i < newPointIdxVector0.size (); ++i){
    cloud_tem->push_back(cloud1->points[newPointIdxVector0[i]]);
  }

  std::vector<PointCloudPtr_RGB_NORMAL> cluster_points0;
  ECE(cloud_tem, cluster_points0);

  for(int i=0; i<cluster_points0.size(); i++){
    if(cluster_points0.at(i)->size()>200){
      appendCloud_RGB_NORMAL(cluster_points0.at(i), result1);
    }
  }

  // Instantiate octree-based point cloud change detection class
  pcl::octree::OctreePointCloudChangeDetector<Point_RGB_NORMAL> octree1 (resolution);

  // Add points from cloudA to octree
  octree1.setInputCloud (cloud1);
  octree1.addPointsFromInputCloud ();

  // Switch octree buffers: This resets octree but keeps previous tree structure in memory.
  octree1.switchBuffers ();

  // Add points from cloudB to octree
  octree1.setInputCloud (cloud0);
  octree1.addPointsFromInputCloud ();

  std::vector<int> newPointIdxVector1;

  // Get vector of point indices from octree voxels which did not exist in previous buffer
  octree1.getPointIndicesFromNewVoxels (newPointIdxVector1);

  cloud_tem->clear();

  std::cout << "Output from getPointIndicesFromNewVoxels:" << std::endl;
  for (size_t i = 0; i < newPointIdxVector1.size (); ++i){
    cloud_tem->push_back(cloud0->points[newPointIdxVector1[i]]);
  }

  std::vector<PointCloudPtr_RGB_NORMAL> cluster_points1;
  ECE(cloud_tem, cluster_points1);

  for(int i=0; i<cluster_points1.size(); i++){
    if(cluster_points1.at(i)->size()>200){
      appendCloud_RGB_NORMAL(cluster_points1.at(i), result0);
    }
  }
}