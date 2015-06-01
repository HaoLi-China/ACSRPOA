#include "visualizer.h"
#include <boost/thread/thread.hpp>

Visualizer::Visualizer(){
  viewer.reset (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->initCameraParameters();
}

Visualizer::~Visualizer(){

}

//void Visualizer::addPointCloud_RGB_NORMAL(PointCloudPtr_RGB_NORMAL cloud, char* id){
//	PointCloudPtr_RGB pc(new PointCloud_RGB);
//  for(int i=0;i<cloud->size();i++){
//    Point_RGB pr(cloud->at(i).x,cloud->at(i).y,cloud->at(i).z);
//    pc->push_back(pr);
//  }
//   viewer->addPointCloud (pc, id);
//}

void Visualizer::show(){
  viewer->addCoordinateSystem(1.0);
  viewer->resetCamera();

  while (!viewer->wasStopped ())
  {
    viewer->spinOnce(100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
}

//show rgb cloud
void showPointCloud (PointCloudPtr_RGB cloud, std::string name)
{
  pcl::visualization::CloudViewer viewer (name);

  viewer.showCloud (cloud);
  while (!viewer.wasStopped ())
  {

  }
}

//show cloud
void showPointCloud2 (PointCloudPtr cloud, std::string name)
{
  pcl::visualization::CloudViewer viewer (name);

  viewer.showCloud (cloud);
  while (!viewer.wasStopped ())
  {

  }
}

//show rgb_normal cloud
void showPointCloud3 (PointCloudPtr_RGB_NORMAL cloud, std::string name){
  PointCloudPtr_RGB cloudrgb(new PointCloud_RGB);

  for(int i=0; i<cloud->size(); i++){
    Point_RGB pt;
    pt.x=cloud->points[i].x;
    pt.y=cloud->points[i].y;
    pt.z=cloud->points[i].z;
    pt.r=cloud->points[i].r;
    pt.g=cloud->points[i].g;
    pt.b=cloud->points[i].b;
    cloudrgb->push_back(pt);
  }

  pcl::visualization::CloudViewer viewer (name);

  viewer.showCloud (cloudrgb);
  while (!viewer.wasStopped ())
  {

  }
}