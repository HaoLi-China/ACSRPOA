#include "scene_seg.h"
#include "supervoxel_clustering.h"
#include <pcl/octree/octree.h>

//for least square
bool getPlaneByLeastSquare(PointCloudPtr_RGB_NORMAL cloud_all_in_plane, pcl::ModelCoefficients::Ptr coefficients)
{
  double coeffA = 0.0, coeffB = 0.0, coeffC = 0.0, coeffD = 0.0;
  int matrixSize = 3;
  Eigen::Matrix3Xd mpara( matrixSize, matrixSize );
  for (int i =0; i<matrixSize;i++){
    for (int j=0; j<matrixSize;j++){
      mpara(i,j) = 0;
    }
  }

  double sumx, sumy, sumz;
  double averx, avery, averz;

  sumx=sumy=sumz=0;

  for(int i=0;i<cloud_all_in_plane->points.size();i++){
    sumx+=cloud_all_in_plane->points.at(i).x;
    sumy+=cloud_all_in_plane->points.at(i).y;
    sumz+=cloud_all_in_plane->points.at(i).z;
  }

  averx=sumx/cloud_all_in_plane->points.size();
  avery=sumy/cloud_all_in_plane->points.size();
  averz=sumz/cloud_all_in_plane->points.size();

  for(int i=0;i<cloud_all_in_plane->points.size();i++){
    mpara( 0, 0 ) += pow( cloud_all_in_plane->points.at(i).x - averx, 2 );
    mpara( 0, 1 ) += ( cloud_all_in_plane->points.at(i).x - averx ) * ( cloud_all_in_plane->points.at(i).y - avery );
    mpara( 0, 2 ) += ( cloud_all_in_plane->points.at(i).x - averx ) * ( cloud_all_in_plane->points.at(i).z - averz );

    mpara( 1, 0 ) += ( cloud_all_in_plane->points.at(i).x - averx ) * ( cloud_all_in_plane->points.at(i).y - avery );
    mpara( 1, 1 ) += pow( cloud_all_in_plane->points.at(i).y - avery, 2 );
    mpara( 1, 2 ) += ( cloud_all_in_plane->points.at(i).y - avery ) * ( cloud_all_in_plane->points.at(i).z - averz );

    mpara( 2, 0 ) += ( cloud_all_in_plane->points.at(i).x - averx ) * ( cloud_all_in_plane->points.at(i).z - averz );
    mpara( 2, 1 ) += ( cloud_all_in_plane->points.at(i).y - avery ) * ( cloud_all_in_plane->points.at(i).z - averz );
    mpara( 2, 2 ) += pow( cloud_all_in_plane->points.at(i).z - averz, 2 );
  }

  Eigen::EigenSolver<Eigen::Matrix3Xd> msolver( mpara );
  complex<double> lambda1 = msolver.eigenvalues()[0];
  complex<double> lambda2 = msolver.eigenvalues()[1];
  complex<double> lambda3 = msolver.eigenvalues()[2];
  int minEigenValue = (( lambda1.real() < lambda2.real() ) ? 0 :1 );
  minEigenValue = (( msolver.eigenvalues()[minEigenValue].real() < lambda3.real() )? minEigenValue : 2);
  coeffA = msolver.eigenvectors().col(minEigenValue)[0].real();
  coeffB = msolver.eigenvectors().col(minEigenValue)[1].real();
  coeffC = msolver.eigenvectors().col(minEigenValue)[2].real();
  coeffD = -( coeffA * averx + coeffB * avery + coeffC * averz );

  cout<<endl;
  cout<<coeffA<<"==========="<<coeffB<<"=============="<<coeffC<<"============"<<coeffD<<endl;

  coefficients->values.push_back(coeffA);
  coefficients->values.push_back(coeffB);
  coefficients->values.push_back(coeffC);
  coefficients->values.push_back(coeffD);

  return true;
}

//compute bounding box
void com_bounding_box(PointCloudPtr_RGB_NORMAL cloud,float *min_x,float *min_y,float *min_z, float *max_x, float *max_y, float *max_z){
  *min_x=cloud->points[0].x;
  *min_y=cloud->points[0].y;
  *min_z=cloud->points[0].z;
  *max_x=cloud->points[0].x;
  *max_y=cloud->points[0].y;
  *max_z=cloud->points[0].z;

  for (int i=0; i<cloud->size(); ++i) {
    float x, y, z;
    x=cloud->points[i].x;
    y=cloud->points[i].y;
    z=cloud->points[i].z;

    if(x<(*min_x)){
      (*min_x)=x;
    }
    else if(x>(*max_x)){
      (*max_x)=x;
    }

    if(y<(*min_y)){
      (*min_y)=y;
    }
    else if(y>(*max_y)){
      (*max_y)=y;
    }

    if(z<(*min_z)){
      (*min_z)=z;
    }
    else if(z>(*max_z)){
      (*max_z)=z;
    }
  }
}

//compute max value, min value, and average value along z axis of the point cloud
void com_max_and_min_and_avg_z(PointCloudPtr_RGB_NORMAL cloud,float *min_z,float *max_z,float *avg_z){
  *min_z=cloud->points[0].z;
  *max_z=cloud->points[0].z;

  float sum_z=0;

  for (int i=0; i<cloud->size(); ++i) {
    float z;

    z=cloud->points[i].z;

    sum_z+=z;

    if(z<(*min_z)){
      (*min_z)=z;
    }
    else if(z>(*max_z)){
      (*max_z)=z;
    }
  }

  *avg_z=sum_z/cloud->size();
}

//find a minimum bounding rect
void find_min_rect(PointCloudPtr_RGB_NORMAL cloud, cv::Point2f &p0,cv::Point2f &p1,cv::Point2f &p2,cv::Point2f &p3){
  std::vector<cv::Point2f> points_clu_2d;

  for(int j=0;j<cloud->points.size();j++){
    points_clu_2d.push_back(cv::Point2f(cloud->points[j].x, cloud->points[j].y));
  }

  cv::RotatedRect rect = cv::minAreaRect(cv::Mat(points_clu_2d));

  float width= rect.size.width;
  float height= rect.size.height;

  p0.x=rect.center.x-width/2.0;
  p0.y=rect.center.y-height/2.0;

  p1.x=rect.center.x-width/2.0;
  p1.y=rect.center.y+height/2.0;

  p2.x=rect.center.x+width/2.0;
  p2.y=rect.center.y+height/2.0;

  p3.x=rect.center.x+width/2.0;
  p3.y=rect.center.y-height/2.0;

  float ang=(rect.angle/180.0)*PII;

  float x0=rect.center.x+(p0.x-rect.center.x)*cos(ang)-(p0.y-rect.center.y)*sin(ang);
  float y0=rect.center.y+(p0.x-rect.center.x)*sin(ang)+(p0.y-rect.center.y)*cos(ang);

  float x1=rect.center.x+(p1.x-rect.center.x)*cos(ang)-(p1.y-rect.center.y)*sin(ang);
  float y1=rect.center.y+(p1.x-rect.center.x)*sin(ang)+(p1.y-rect.center.y)*cos(ang);

  float x2=rect.center.x+(p2.x-rect.center.x)*cos(ang)-(p2.y-rect.center.y)*sin(ang);
  float y2=rect.center.y+(p2.x-rect.center.x)*sin(ang)+(p2.y-rect.center.y)*cos(ang);

  float x3=rect.center.x+(p3.x-rect.center.x)*cos(ang)-(p3.y-rect.center.y)*sin(ang);
  float y3=rect.center.y+(p3.x-rect.center.x)*sin(ang)+(p3.y-rect.center.y)*cos(ang);

  p0.x=x0;
  p0.y=y0;

  p1.x=x1;
  p1.y=y1;

  p2.x=x2;
  p2.y=y2;

  p3.x=x3;
  p3.y=y3;
}

//get Rect For PlaneCloud
void getRectForPlaneCloud(PointCloudPtr_RGB_NORMAL plane_cloud, pcl::ModelCoefficients::Ptr plane_coefficients, PointCloudPtr rect_cloud){
  PointCloudPtr_RGB_NORMAL cloud_in_plane(new PointCloud_RGB_NORMAL);

  PointCloudPtr_RGB_NORMAL plane_cloud_tem(new PointCloud_RGB_NORMAL);
  pcl::copyPointCloud(*plane_cloud,*plane_cloud_tem);

  Point_RGB_NORMAL pr;
  pr.x=0;
  pr.y=0;
  pr.z=(-plane_coefficients->values[3])/plane_coefficients->values[2];

  plane_cloud_tem->push_back(pr);

  Eigen::Vector3d plane_normal;
  plane_normal << plane_coefficients->values[0], plane_coefficients->values[1], plane_coefficients->values[2];
  plane_normal.normalize();

  double angle=acos(plane_normal.dot(Eigen::Vector3d(0,0,1)));
  Eigen::Vector3d axis=plane_normal.cross(Eigen::Vector3d(0,0,1));
  axis.normalize();

  Eigen::Matrix4d matrix;
  getRotationMatrix(axis, angle, matrix);

  Eigen::Matrix4f matrix_transform = matrix.cast<float>();
  pcl::transformPointCloud (*plane_cloud_tem, *cloud_in_plane, matrix_transform);

  Point_RGB_NORMAL new_pr=cloud_in_plane->at(cloud_in_plane->size()-1);

  pointCloud_RGB_NORMAPopUp(cloud_in_plane);

  cv::Point2f p0;
  cv::Point2f p1;
  cv::Point2f p2;
  cv::Point2f p3;

  std::cout<<"cloud_in_plane->size:"<< cloud_in_plane->size() <<std::endl;
  find_min_rect(cloud_in_plane, p0,p1,p2,p3);

  /*float min_z,max_z,avg_z;
  com_max_and_min_and_avg_z(cloud_in_plane,&min_z,&max_z,&avg_z);

  float cloud_z=min_z;

  if(max_z-avg_z<avg_z-min_z){
  cloud_z=max_z;
  }*/

  PointCloudPtr points(new PointCloud());
  points->push_back(Point(p0.x,p0.y,new_pr.z));
  points->push_back(Point(p1.x,p1.y,new_pr.z));
  points->push_back(Point(p2.x,p2.y,new_pr.z));
  points->push_back(Point(p3.x,p3.y,new_pr.z));

  Eigen::Matrix4d matrix_reverse;
  getRotationMatrix(axis, -angle, matrix_reverse);

  Eigen::Matrix4f matrix_transform_reverse = matrix_reverse.cast<float>();
  pcl::transformPointCloud (*points, *rect_cloud, matrix_transform_reverse);
}


//big plane fitting
void big_plane_fitting(PointCloudPtr_RGB_NORMAL sourceCloud, MyPointCloud_RGB_NORMAL &plane_cloud, MyPointCloud &rect_cloud, pcl::ModelCoefficients::Ptr plane_coefficients, PointCloudPtr_RGB_NORMAL remained_cloud){
  PointCloudPtr_RGB_NORMAL cloud_tem(new PointCloud_RGB_NORMAL);
  pcl::copyPointCloud(*sourceCloud,*cloud_tem);

  //pcl::SACSegmentationFromNormals<Point_RGB_NORMAL, pcl::Normal> seg; 
  //pcl::ExtractIndices<Point_RGB_NORMAL> extract;
  //pcl::search::KdTree<Point_RGB_NORMAL>::Ptr tree (new pcl::search::KdTree<Point_RGB_NORMAL> ());
  //pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

  //pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  //PointCloudPtr_RGB_NORMAL cloud_p (new PointCloud_RGB_NORMAL);
  //PointCloudPtr_RGB_NORMAL cloud_f (new PointCloud_RGB_NORMAL);

  //for(int i=0; i<cloud_tem->size(); i++){
  //  pcl::Normal nor;
  //  nor.normal_x=cloud_tem->at(i).normal_x;
  //  nor.normal_y=cloud_tem->at(i).normal_y;
  //  nor.normal_z=cloud_tem->at(i).normal_z;
  //  cloud_normals->push_back(nor);
  //}

  //// Create the segmentation object for the planar model and set all the parameters
  //seg.setOptimizeCoefficients (true);
  //seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  //seg.setNormalDistanceWeight (0.1);
  //seg.setMethodType (pcl::SAC_RANSAC);
  //seg.setMaxIterations (100);
  //seg.setDistanceThreshold (0.02);
  //seg.setInputCloud (cloud_tem);
  //seg.setInputNormals (cloud_normals);
  //// Obtain the plane inliers and coefficients
  //seg.segment (*inliers, *plane_coefficients);

  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  PointCloudPtr_RGB_NORMAL cloud_p (new PointCloud_RGB_NORMAL);
  PointCloudPtr_RGB_NORMAL cloud_f (new PointCloud_RGB_NORMAL);
  pcl::ExtractIndices<Point_RGB_NORMAL> extract;

  pcl::SACSegmentation<Point_RGB_NORMAL> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (0.015);

  // Segment the largest planar component from the remaining cloud
  seg.setInputCloud (cloud_tem);
  seg.segment (*inliers, *plane_coefficients);

  if (inliers->indices.size () < 1000)
  {
    std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    pcl::copyPointCloud(*cloud_tem,*remained_cloud);
    return;
  }

  // Extract the inliers
  extract.setInputCloud (cloud_tem);
  extract.setIndices (inliers);
  extract.setNegative (false);
  extract.filter (*cloud_p);
  std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

  PointCloudPtr rect_cl(new PointCloud);
  getRectForPlaneCloud(cloud_p, plane_coefficients, rect_cl);
  PointCloud_RGB_NORMAL2MyPointCloud_RGB_NORMAL(cloud_p, plane_cloud);
  PointCloud2MyPointCloud(rect_cl, rect_cloud);

  // Create the filtering object
  extract.setNegative (true);
  extract.filter (*cloud_f);
  cloud_tem.swap (cloud_f);

  pcl::copyPointCloud(*cloud_tem,*remained_cloud);
}

//detect table
void detect_table(PointCloudPtr_RGB_NORMAL sourceCloud, pcl::ModelCoefficients& plane_coefficients, PointCloudPtr_RGB_NORMAL planeCloud, PointCloudPtr rect_cloud, PointCloudPtr_RGB_NORMAL remainingCloud){
  pcl::ExtractIndices<Point_RGB_NORMAL> extract;// Create the filtering object
  PointCloudPtr_RGB_NORMAL cloud_tem(new PointCloud_RGB_NORMAL);
  PointCloudPtr_RGB_NORMAL cloud_p(new PointCloud_RGB_NORMAL);

  pcl::copyPointCloud(*sourceCloud, *cloud_tem);

  while(1){
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<Point_RGB_NORMAL> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (1000);
    seg.setDistanceThreshold (0.015);

    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_tem);
    seg.segment (*inliers, plane_coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
      return;
    }

    //showPointCloud(cloud_tem, "cloud_tem");

    // Extract the inliers
    extract.setInputCloud (cloud_tem);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*planeCloud);
    std::cerr << "PointCloud representing the planar component: " << planeCloud->width * planeCloud->height << " data points." << std::endl;

    //showPointCloud(planeCloud, "planeCloud");

    getRectForPlaneCloud(planeCloud, boost::make_shared<pcl::ModelCoefficients>(plane_coefficients), rect_cloud);

    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*cloud_p);
    //showPointCloud(cloud_p, "cloud_p");

    pcl::copyPointCloud(*cloud_p, *cloud_tem);

    Eigen::Vector3d normal;
    normal << plane_coefficients.values[0], plane_coefficients.values[1], plane_coefficients.values[2];
    normal.normalize();

    if(std::abs(normal.dot(Eigen::Vector3d(0,0,1)))>0.7){
      break;
    }
    else{
      appendCloud_RGB_NORMAL(planeCloud, remainingCloud);
    }
  }

  appendCloud_RGB_NORMAL(cloud_tem, remainingCloud);
}

//get transform matrix between plane and x_y plane 
void getTemTransformMatrix(pcl::ModelCoefficients& coefficients, Eigen::Matrix4f& matrix_transform, Eigen::Matrix4f& matrix_transform_r){
  Eigen::Vector3d normal;
  normal << coefficients.values[0], coefficients.values[1], coefficients.values[2];
  normal.normalize();

  if(normal.dot(Eigen::Vector3d(0,0,1))<0){
    normal = -normal;
  }

  double angle=acos(normal.dot(Eigen::Vector3d(0,0,1)));
  Eigen::Vector3d axis=normal.cross(Eigen::Vector3d(0,0,1));
  axis.normalize();

  Eigen::Matrix4d matrix;
  getRotationMatrix(axis, angle, matrix);
  matrix_transform = matrix.cast<float>();

  getRotationMatrix(axis, -angle, matrix);
  matrix_transform_r = matrix.cast<float>();
}

//Euclidean Cluster Extraction
void object_seg_ECE(PointCloudPtr_RGB_NORMAL cloud, std::vector<MyPointCloud_RGB_NORMAL> &cluster_points){
  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<Point_RGB_NORMAL>::Ptr tree (new pcl::search::KdTree<Point_RGB_NORMAL>);
  tree->setInputCloud (cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<Point_RGB_NORMAL> ec;
  ec.setClusterTolerance (0.010); // 2cm
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

    MyPointCloud_RGB_NORMAL mpc;
    PointCloud_RGB_NORMAL2MyPointCloud_RGB_NORMAL(cloud_cluster, mpc);
    cluster_points.push_back(mpc);

    j++;
  }
}

//Find Points In Cylinder
void findPointsIn_Cylinder(PointCloudPtr_RGB_NORMAL cloud, pcl::ModelCoefficients::Ptr &coefficients, pcl::PointIndices::Ptr &inliers){

  Eigen::Vector3d cylinder_normal;
  cylinder_normal << coefficients->values[3], coefficients->values[4], coefficients->values[5];
  cylinder_normal.normalize();

  double min=0;
  double max=0;
  bool flag=true;

  for(int i=0;i<cloud->size();i++){
    double x=cloud->points[i].x;
    double y=cloud->points[i].y;
    double z=cloud->points[i].z;

    double distance=sqrt(pow(coefficients->values[0]-x, 2)+pow(coefficients->values[1]-y, 2)+pow(coefficients->values[2]-z, 2));

    Eigen::Vector3d tem_normal;
    tem_normal << x-coefficients->values[0], y-coefficients->values[1], z-coefficients->values[2];
    tem_normal.normalize();

    double angle=acos(cylinder_normal.dot(tem_normal));
    //double angle1=std::abs(acos(cylinder_normal.dot(Eigen::Vector3d(0,0,-1)))-PI/2);
    //double dis_z=coefficients->values[6]*cos(angle1);

    if(std::abs(std::abs(distance*sin(angle))-coefficients->values[6])<=0.005){

      double tem_normal_dis=distance*(cylinder_normal.dot(tem_normal));
      double tem_cen_z=coefficients->values[2]+tem_normal_dis*(cylinder_normal.dot(Eigen::Vector3d(0,0,1)));

      if(flag){
        min=tem_normal_dis;
        max=tem_normal_dis;
        flag=false;
      }

      if(tem_normal_dis<min){
        if(tem_cen_z>0){
          min=tem_normal_dis;
        }
      }

      if(tem_normal_dis>max){
        if(tem_cen_z>0){
          max=tem_normal_dis;
        }
      }
    }
  }

  if(min!=max){

    for(int i=0;i<cloud->size();i++){
      double x=cloud->points[i].x;
      double y=cloud->points[i].y;
      double z=cloud->points[i].z;

      double distance=sqrt(pow(coefficients->values[0]-x, 2)+pow(coefficients->values[1]-y, 2)+pow(coefficients->values[2]-z, 2));

      Eigen::Vector3d tem_normal;
      tem_normal << x-coefficients->values[0], y-coefficients->values[1], z-coefficients->values[2];
      tem_normal.normalize();

      double angle=acos(cylinder_normal.dot(tem_normal));

      if(std::abs(distance*sin(angle))<=(coefficients->values[6]+0.01)){
        if(distance*(cylinder_normal.dot(tem_normal))<=max&&distance*(cylinder_normal.dot(tem_normal))>=min){
          inliers->indices.push_back(i);
        }
      }
    }

    coefficients->values.push_back(coefficients->values[0]+max*(cylinder_normal.dot(Eigen::Vector3d(1,0,0))));
    coefficients->values.push_back(coefficients->values[1]+max*(cylinder_normal.dot(Eigen::Vector3d(0,1,0))));
    coefficients->values.push_back(coefficients->values[2]+max*(cylinder_normal.dot(Eigen::Vector3d(0,0,1))));

    coefficients->values[0]+=min*(cylinder_normal.dot(Eigen::Vector3d(1,0,0)));
    coefficients->values[1]+=min*(cylinder_normal.dot(Eigen::Vector3d(0,1,0)));
    coefficients->values[2]+=min*(cylinder_normal.dot(Eigen::Vector3d(0,0,1)));

    std::cerr << "Cylinder coefficients==: " << *coefficients << std::endl;
  }
}

//cylinder fitting
void cylinder_fitting(PointCloudPtr_RGB_NORMAL cloud, MyPointCloud_RGB_NORMAL &cylinder_cloud, pcl::ModelCoefficients::Ptr cylinder_coefficients,PointCloudPtr_RGB_NORMAL remained_cloud){
  // All the objects needed
  pcl::PassThrough<Point_RGB_NORMAL> pass;

  // Datasets
  pcl::PointCloud<Point_RGB_NORMAL>::Ptr cloud_filtered (new pcl::PointCloud<Point_RGB_NORMAL>);
  pcl::PointCloud<Point_RGB_NORMAL>::Ptr cloud_f (new pcl::PointCloud<Point_RGB_NORMAL>);

  // Build a passthrough filter to remove spurious NaNs
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0, 1.0);
  pass.filter (*cloud_filtered);
  std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size () << " data points." << std::endl;

  pcl::SACSegmentationFromNormals<Point_RGB_NORMAL, pcl::Normal> seg;

  pcl::ExtractIndices<Point_RGB_NORMAL> extract;
  pcl::NormalEstimation<Point_RGB_NORMAL, pcl::Normal> ne;
  pcl::search::KdTree<Point_RGB_NORMAL>::Ptr tree (new pcl::search::KdTree<Point_RGB_NORMAL> ());

  pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices);

  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

  // Estimate point normals
  ne.setSearchMethod (tree);
  ne.setInputCloud (cloud_filtered);
  ne.setKSearch (50);
  ne.compute (*cloud_normals);

  // Create the segmentation object for cylinder segmentation and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_CYLINDER);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setNormalDistanceWeight (0.1);
  seg.setMaxIterations (1000);
  //seg.setDistanceThreshold (0.04);
  seg.setDistanceThreshold (0.025);
  seg.setRadiusLimits (0.01, 0.1);
  seg.setInputCloud (cloud_filtered);
  seg.setInputNormals (cloud_normals);

  // Obtain the cylinder inliers and coefficients
  seg.segment (*inliers_cylinder, *cylinder_coefficients);

  extract.setInputCloud (cloud_filtered);
  extract.setIndices (inliers_cylinder);
  extract.setNegative (false);
  PointCloudPtr_RGB_NORMAL cloud_cylinder (new PointCloud_RGB_NORMAL());
  extract.filter (*cloud_cylinder);
  if (cloud_cylinder->points.size()<500){
    std::cerr << "Can't find the cylindrical component." << std::endl;
  }
  else
  {
    std::cerr << "PointCloud representing the cylindrical component: " << cloud_cylinder->points.size () << " data points." << std::endl;

    pcl::PointIndices::Ptr tem_inliers(new pcl::PointIndices);
    findPointsIn_Cylinder(cloud_filtered, cylinder_coefficients, tem_inliers);

    std::cout<<"inliers->indices.size:"<< tem_inliers->indices.size() <<std::endl;

    double h=sqrt(pow(cylinder_coefficients->values[0]-cylinder_coefficients->values[7], 2)+
      pow(cylinder_coefficients->values[1]-cylinder_coefficients->values[8], 2)+
      pow(cylinder_coefficients->values[2]-cylinder_coefficients->values[9], 2));


    if(tem_inliers->indices.size()==0||tem_inliers->indices.size()>10000||h>0.5){
    }
    else{
      extract.setIndices (tem_inliers);
      extract.setNegative (false);
      extract.filter (*cloud_f);

      PointCloud_RGB_NORMAL2MyPointCloud_RGB_NORMAL(cloud_f, cylinder_cloud);


      extract.setIndices (tem_inliers);
      extract.setNegative (true);
      extract.filter (*cloud_f);
      cloud_filtered.swap (cloud_f);
    }
  }

  pcl::copyPointCloud(*cloud_filtered,*remained_cloud);
}

//segment cylinder from the data
void object_seg_Cylinder(PointCloudPtr_RGB_NORMAL cloud, std::vector<MyPointCloud_RGB_NORMAL> &cluster_points, std::vector<pcl::ModelCoefficients> &coefficients,PointCloudPtr_RGB_NORMAL remained_cloud){
  // All the objects needed
  pcl::PassThrough<Point_RGB_NORMAL> pass;

  // Datasets
  pcl::PointCloud<Point_RGB_NORMAL>::Ptr cloud_filtered (new pcl::PointCloud<Point_RGB_NORMAL>);
  pcl::PointCloud<Point_RGB_NORMAL>::Ptr cloud_filtered_tem (new pcl::PointCloud<Point_RGB_NORMAL>);
  pcl::PointCloud<Point_RGB_NORMAL>::Ptr cloud_f (new pcl::PointCloud<Point_RGB_NORMAL>);

  // Build a passthrough filter to remove spurious NaNs
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0, 1.0);
  pass.filter (*cloud_filtered);
  std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size () << " data points." << std::endl;

  //pcl::copyPointCloud(*cloud_filtered,*cloud_filtered_tem);

  int d=0;//debug

  do{
    pcl::SACSegmentationFromNormals<Point_RGB_NORMAL, pcl::Normal> seg;

    pcl::ExtractIndices<Point_RGB_NORMAL> extract;
    pcl::NormalEstimation<Point_RGB_NORMAL, pcl::Normal> ne;
    pcl::search::KdTree<Point_RGB_NORMAL>::Ptr tree (new pcl::search::KdTree<Point_RGB_NORMAL> ());

    pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices);

    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

    // Estimate point normals
    ne.setSearchMethod (tree);
    ne.setInputCloud (cloud_filtered);
    ne.setKSearch (50);
    ne.compute (*cloud_normals);

    // Create the segmentation object for cylinder segmentation and set all the parameters
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_CYLINDER);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight (0.1);
    seg.setMaxIterations (10000);
    //seg.setDistanceThreshold (0.04);
    seg.setDistanceThreshold (0.025);
    seg.setRadiusLimits (0.01, 0.1);
    seg.setInputCloud (cloud_filtered);
    seg.setInputNormals (cloud_normals);

    // Obtain the cylinder inliers and coefficients
    seg.segment (*inliers_cylinder, *coefficients_cylinder);
    //std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;
    std::cout<< "seg.getProbability():" << seg.getProbability() <<std::endl;

    /*if(d==1){
    showPointClound (cloud_filtered,"0");
    }*/

    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers_cylinder);
    extract.setNegative (false);
    pcl::PointCloud<Point_RGB_NORMAL>::Ptr cloud_cylinder (new pcl::PointCloud<Point_RGB_NORMAL> ());
    extract.filter (*cloud_cylinder);
    if (cloud_cylinder->points.size()<500){
      appendCloud_RGB_NORMAL(cloud_filtered,cloud_filtered_tem);
      std::cerr << "Can't find the cylindrical component." << std::endl;
      break;
    }
    else
    {
      std::cerr << "PointCloud representing the cylindrical component: " << cloud_cylinder->points.size () << " data points." << std::endl;

      pcl::PointIndices::Ptr tem_inliers(new pcl::PointIndices);
      findPointsIn_Cylinder(cloud_filtered, coefficients_cylinder, tem_inliers);

      std::cout<<"inliers->indices.size:"<< tem_inliers->indices.size() <<std::endl;

      double h=sqrt(pow(coefficients_cylinder->values[0]-coefficients_cylinder->values[7], 2)+
        pow(coefficients_cylinder->values[1]-coefficients_cylinder->values[8], 2)+
        pow(coefficients_cylinder->values[2]-coefficients_cylinder->values[9], 2));


      if(tem_inliers->indices.size()==0||tem_inliers->indices.size()>10000||h>0.5){
        extract.setIndices (inliers_cylinder);
        extract.setNegative (false);
        extract.filter (*cloud_f);

        appendCloud_RGB_NORMAL(cloud_f,cloud_filtered_tem);

        extract.setNegative (true);
        extract.filter (*cloud_f);
        cloud_filtered.swap (cloud_f);
      }
      else{
        double l=2*PII*coefficients_cylinder->values[6];
        double rate=inliers_cylinder->indices.size()/(h*l);
        cout<<"rate_cylinder========================:"<< rate <<endl;

        if(rate>10000){
          extract.setIndices (tem_inliers);
          extract.setNegative (false);
          extract.filter (*cloud_f);

          MyPointCloud_RGB_NORMAL mpc;
          PointCloud_RGB_NORMAL2MyPointCloud_RGB_NORMAL(cloud_f, mpc);
          cluster_points.push_back(mpc);

          extract.setIndices (tem_inliers);
          extract.setNegative (true);
          extract.filter (*cloud_f);
          cloud_filtered.swap (cloud_f);

          //showPointClound (cloud_filtered,"1");

          //cluster_inliers.push_back(*tem_inliers);

          /*coefficients.push_back(*coefficients_cylinder);
          extract.setIndices (tem_inliers);
          extract.setInputCloud (cloud_filtered_tem);
          extract.setNegative (true);
          extract.filter (*cloud_f);
          cloud_filtered_tem.swap (cloud_f);*/

          d++;
        }
        else{
          extract.setIndices (inliers_cylinder);
          extract.setNegative (false);
          extract.filter (*cloud_f);

          appendCloud_RGB_NORMAL(cloud_f,cloud_filtered_tem);

          extract.setIndices (inliers_cylinder);
          extract.setNegative (true);
          extract.filter (*cloud_f);
          cloud_filtered.swap (cloud_f);
        }
      }

      //extract.setInputCloud (cloud_filtered);
    }

  }while(1);

  cout<<"coefficients.size()++++++++++++++++++++:"<<coefficients.size()<<endl;
  pcl::copyPointCloud(*cloud_filtered_tem,*remained_cloud);
}


//Find Points In Sphere
void findPointsIn_Sphere(PointCloudPtr_RGB_NORMAL cloud, pcl::ModelCoefficients::Ptr &coefficients, pcl::PointIndices::Ptr &inliers){
  for(int i=0;i<cloud->size();i++){
    float x=cloud->points[i].x;
    float y=cloud->points[i].y;
    float z=cloud->points[i].z;

    float dist=sqrt(pow((x-coefficients->values[0]),2)+pow((y-coefficients->values[1]),2)+pow((z-coefficients->values[2]),2));

    if(dist<=coefficients->values[3]+0.01){
      inliers->indices.push_back(i);
    }
  }
}

//sphere fitting
void sphere_fitting(PointCloudPtr_RGB_NORMAL cloud, MyPointCloud_RGB_NORMAL &sphere_cloud, pcl::ModelCoefficients::Ptr sphere_coefficients, PointCloudPtr_RGB_NORMAL remained_cloud){
  // Datasets
  PointCloudPtr_RGB_NORMAL cloud_filtered (new PointCloud_RGB_NORMAL);
  PointCloudPtr_RGB_NORMAL cloud_f (new PointCloud_RGB_NORMAL);

  pcl::copyPointCloud(*cloud,*cloud_filtered);

  // Create the segmentation object
  pcl::SACSegmentation<Point_RGB_NORMAL> seg;
  pcl::PointIndices::Ptr inliers_sphere(new pcl::PointIndices);
  pcl::ExtractIndices<Point_RGB_NORMAL> extract;

  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_SPHERE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (0.02);

  // Segment the sphere component from the remaining cloud
  seg.setInputCloud (cloud_filtered);
  seg.segment (*inliers_sphere, *sphere_coefficients);


  extract.setInputCloud (cloud_filtered);
  extract.setIndices (inliers_sphere);
  extract.setNegative (false);
  pcl::PointCloud<Point_RGB_NORMAL>::Ptr cloud_sphere (new pcl::PointCloud<Point_RGB_NORMAL> ());
  extract.filter (*cloud_sphere);

  double rate=0;

  if (inliers_sphere->indices.size () < 500)
  {
    std::cerr << "Could not estimate a sphere model for the given dataset." << std::endl;
    pcl::copyPointCloud(*cloud_filtered,*remained_cloud);
    return;
  }

  if(sphere_coefficients->values[3]>0.5||sphere_coefficients->values[2]-sphere_coefficients->values[3]<0){
    std::cerr << "Could not estimate a sphere model for the given dataset===." << std::endl;
    pcl::copyPointCloud(*cloud_filtered,*remained_cloud);
    return;
  }
  else{
    std::cout<< "coefficients_sphere:" <<  *sphere_coefficients <<std::endl;
    pcl::PointIndices::Ptr tem_inliers(new pcl::PointIndices);
    findPointsIn_Sphere(cloud_filtered, sphere_coefficients, tem_inliers);


    extract.setIndices (tem_inliers);
    extract.setNegative (false);
    extract.filter (*cloud_f);

    PointCloud_RGB_NORMAL2MyPointCloud_RGB_NORMAL(cloud_f, sphere_cloud);

    extract.setIndices (tem_inliers);
    extract.setNegative (true);
    extract.filter (*cloud_f);
    cloud_filtered.swap (cloud_f);
  }

  pcl::copyPointCloud(*cloud_filtered,*remained_cloud);
}

//segment sphere from the data
void object_seg_Sphere(PointCloudPtr_RGB_NORMAL cloud, std::vector<MyPointCloud_RGB_NORMAL> &cluster_points, std::vector<pcl::ModelCoefficients> &coefficients,PointCloudPtr_RGB_NORMAL remained_cloud){

  // Datasets
  PointCloudPtr_RGB_NORMAL cloud_filtered (new PointCloud_RGB_NORMAL);
  PointCloudPtr_RGB_NORMAL cloud_filtered_tem (new PointCloud_RGB_NORMAL);
  PointCloudPtr_RGB_NORMAL cloud_f (new PointCloud_RGB_NORMAL);

  //pcl::copyPointCloud(*cloud,*cloud_filtered);
  pcl::copyPointCloud(*cloud,*cloud_filtered_tem);

  while(1){
    // Create the segmentation object
    pcl::SACSegmentation<Point_RGB_NORMAL> seg;
    pcl::ModelCoefficients::Ptr coefficients_sphere (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_sphere(new pcl::PointIndices);
    pcl::ExtractIndices<Point_RGB_NORMAL> extract;

    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_SPHERE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (10000);
    seg.setDistanceThreshold (0.1);

    // Segment the sphere component from the remaining cloud
    seg.setInputCloud (cloud_filtered_tem);
    seg.segment (*inliers_sphere, *coefficients_sphere);

    std::cout<< "sphere_seg.getProbability():" <<seg.getProbability()<<std::endl;

    extract.setInputCloud (cloud_filtered_tem);
    extract.setIndices (inliers_sphere);
    extract.setNegative (false);
    pcl::PointCloud<Point_RGB_NORMAL>::Ptr cloud_sphere (new pcl::PointCloud<Point_RGB_NORMAL> ());
    extract.filter (*cloud_sphere);

    double rate=0;

    if (inliers_sphere->indices.size () < 500)
    {
      appendCloud_RGB_NORMAL(cloud_filtered_tem,cloud_filtered);
      std::cerr << "Could not estimate a sphere model for the given dataset." << std::endl;
      break;
    }
    else{
      double area=4*PII*coefficients_sphere->values[3]*coefficients_sphere->values[3];
      rate=inliers_sphere->indices.size()/area;
      cout<<"rate_sphere========================:"<< rate <<endl;
    }

    if(coefficients_sphere->values[3]>0.5||coefficients_sphere->values[2]-coefficients_sphere->values[3]<0||rate<5000){
      std::cerr << "Could not estimate a sphere model for the given dataset===." << std::endl;
      extract.setIndices (inliers_sphere);
      extract.setNegative (false);
      extract.filter (*cloud_f);

      appendCloud_RGB_NORMAL(cloud_f,cloud_filtered);

      extract.setIndices (inliers_sphere);
      extract.setNegative (true);
      extract.filter (*cloud_f);
      cloud_filtered_tem.swap (cloud_f);
    }
    else{
      coefficients.push_back(*coefficients_sphere);
      std::cout<< "coefficients_sphere:" <<  *coefficients_sphere <<std::endl;
      pcl::PointIndices::Ptr tem_inliers(new pcl::PointIndices);
      findPointsIn_Sphere(cloud_filtered_tem, coefficients_sphere, tem_inliers);

      //cluster_inliers.push_back(*tem_inliers);

      extract.setIndices (tem_inliers);
      extract.setNegative (false);
      extract.filter (*cloud_f);

      MyPointCloud_RGB_NORMAL mpc;
      PointCloud_RGB_NORMAL2MyPointCloud_RGB_NORMAL(cloud_f, mpc);
      cluster_points.push_back(mpc);

      extract.setIndices (tem_inliers);
      extract.setNegative (true);
      extract.filter (*cloud_f);
      cloud_filtered_tem.swap (cloud_f);

      /*extract.setInputCloud(cloud_filtered);
      extract.setIndices (tem_inliers);
      extract.setNegative (true);
      extract.filter (*cloud_f);
      cloud_filtered.swap (cloud_f);*/
    }
  }
  pcl::copyPointCloud(*cloud_filtered,*remained_cloud);

  //showPointClound (cloud_filtered,"sssss");
}

//plane for boxs fitting
void plane_for_boxs_fitting(PointCloudPtr_RGB_NORMAL sourceCloud, MyPointCloud_RGB_NORMAL &plane_cloud, MyPointCloud &rect_cloud, pcl::ModelCoefficients::Ptr plane_coefficients, PointCloudPtr_RGB_NORMAL remained_cloud){
  PointCloudPtr_RGB_NORMAL cloud_tem(new PointCloud_RGB_NORMAL);
  pcl::copyPointCloud(*sourceCloud,*cloud_tem);

  PointCloudPtr_RGB_NORMAL cloud_p (new PointCloud_RGB_NORMAL);
  PointCloudPtr_RGB_NORMAL cloud_f (new PointCloud_RGB_NORMAL);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  pcl::ExtractIndices<Point_RGB_NORMAL> extract;

  // Create the segmentation object
  pcl::SACSegmentation<Point_RGB_NORMAL> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (0.005);

  // Segment the largest planar component from the remaining cloud
  seg.setInputCloud (cloud_tem);
  seg.segment (*inliers, *plane_coefficients);
  if (inliers->indices.size () == 0)
  {
    std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    pcl::copyPointCloud(*cloud_tem,*remained_cloud);
    return;
  }

  // Extract the inliers
  extract.setInputCloud (cloud_tem);
  extract.setIndices (inliers);
  extract.setNegative (false);
  extract.filter (*cloud_p);
  std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

  double rate=0;

  if(cloud_p->size()<200){
    std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    pcl::copyPointCloud(*cloud_tem,*remained_cloud);
    return;
  }
  /* else{
  PointCloudPtr_RGB_NORMAL cloud_in_plane(new PointCloud_RGB());

  Eigen::Vector3d plane_normal;
  plane_normal << plane_coefficients->values[0], plane_coefficients->values[1], plane_coefficients->values[2];
  plane_normal.normalize();

  double angle=acos(plane_normal.dot(Eigen::Vector3d(0,0,1)));
  Eigen::Vector3d axis=plane_normal.cross(Eigen::Vector3d(0,0,1));
  axis.normalize();

  Eigen::Matrix4d matrix;
  getRotationMatrix(axis, angle, matrix);

  Eigen::Matrix4f matrix_transform = matrix.cast<float>();
  pcl::transformPointCloud (*cloud_p, *cloud_in_plane, matrix_transform);

  cv::Point2f p0;
  cv::Point2f p1;
  cv::Point2f p2;
  cv::Point2f p3;

  find_min_rect(cloud_in_plane, p0, p1, p2, p3);

  float a=sqrt(pow((p0.x-p1.x),2)+pow((p0.y-p1.y),2));
  float b=sqrt(pow((p2.x-p1.x),2)+pow((p2.y-p1.y),2));
  double area=a*b;

  rate=cloud_p->size()/area;

  cout<<"rate_plane===============::"<<rate<<endl;
  }

  if(rate>30000){*/
  PointCloudPtr rect_cl(new PointCloud);
  getRectForPlaneCloud(cloud_p, plane_coefficients, rect_cl);
  PointCloud_RGB_NORMAL2MyPointCloud_RGB_NORMAL(cloud_p, plane_cloud);
  PointCloud2MyPointCloud(rect_cl, rect_cloud);

  // Create the filtering object
  extract.setNegative (true);
  extract.filter (*cloud_f);
  cloud_tem.swap (cloud_f);
  // }

  pcl::copyPointCloud(*cloud_tem,*remained_cloud);
}

void detect_plane_for_boxs(PointCloudPtr_RGB_NORMAL sourceCloud, std::vector<pcl::ModelCoefficients> &coefficients_vector, std::vector<MyPointCloud_RGB_NORMAL> &plane_points,PointCloudPtr_RGB_NORMAL remained_cloud){

  PointCloudPtr_RGB_NORMAL clound_tem(new PointCloud_RGB_NORMAL);
  PointCloudPtr_RGB_NORMAL cloud_remaining(new PointCloud_RGB_NORMAL);
  pcl::copyPointCloud(*sourceCloud,*clound_tem);

  PointCloudPtr_RGB_NORMAL cloud_p (new PointCloud_RGB_NORMAL);
  PointCloudPtr_RGB_NORMAL cloud_f (new PointCloud_RGB_NORMAL);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::ExtractIndices<Point_RGB_NORMAL> extract;

  // Create the segmentation object
  pcl::SACSegmentation<Point_RGB_NORMAL> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (0.005);


  while (1)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (clound_tem);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      appendCloud_RGB_NORMAL(clound_tem,cloud_remaining);

      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the inliers
    extract.setInputCloud (clound_tem);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_p);
    std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

    double rate=0;

    if(cloud_p->size()<200){
      appendCloud_RGB_NORMAL(clound_tem,cloud_remaining);
      break;
    }
    else{
      PointCloudPtr_RGB_NORMAL cloud_in_plane(new PointCloud_RGB_NORMAL());
      //pcl::copyPointCloud(*,*cloud_in_plane);

      Eigen::Vector3d plane_normal;
      plane_normal << coefficients->values[0], coefficients->values[1], coefficients->values[2];
      plane_normal.normalize();

      double angle=acos(plane_normal.dot(Eigen::Vector3d(0,0,1)));
      Eigen::Vector3d axis=plane_normal.cross(Eigen::Vector3d(0,0,1));
      axis.normalize();

      Eigen::Matrix4d matrix;
      getRotationMatrix(axis, angle, matrix);

      Eigen::Matrix4f matrix_transform = matrix.cast<float>();
      pcl::transformPointCloud (*cloud_p, *cloud_in_plane, matrix_transform);

      cv::Point2f p0;
      cv::Point2f p1;
      cv::Point2f p2;
      cv::Point2f p3;

      find_min_rect(cloud_in_plane, p0, p1, p2, p3);

      float a=sqrt(pow((p0.x-p1.x),2)+pow((p0.y-p1.y),2));
      float b=sqrt(pow((p2.x-p1.x),2)+pow((p2.y-p1.y),2));
      double area=a*b;

      rate=cloud_p->size()/area;

      cout<<"rate_plane===============::"<<rate<<endl;
    }

    if(rate>30000){
      coefficients_vector.push_back(*coefficients);

      MyPointCloud_RGB_NORMAL mpc;
      PointCloud_RGB_NORMAL2MyPointCloud_RGB_NORMAL(cloud_p, mpc);
      plane_points.push_back(mpc);
    }
    else{
      extract.setNegative (false);
      extract.filter (*cloud_f);

      appendCloud_RGB_NORMAL(cloud_f,cloud_remaining);
    }

    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*cloud_f);
    clound_tem.swap (cloud_f);
  }

  pcl::copyPointCloud(*cloud_remaining,*remained_cloud);
}

//Find Points In Box
void findPointsIn_Box(PointCloudPtr_RGB_NORMAL cloud, PointCloudPtr box_points, pcl::PointIndices::Ptr &inliers){

  Eigen::Vector3d box_plane_normal0;
  Eigen::Vector3d box_plane_normal1;
  Eigen::Vector3d box_plane_normal2;
  box_plane_normal0 << box_points->points[0].x-box_points->points[1].x,box_points->points[0].y-box_points->points[1].y,box_points->points[0].z-box_points->points[1].z;
  box_plane_normal1 << box_points->points[1].x-box_points->points[2].x,box_points->points[1].y-box_points->points[2].y,box_points->points[1].z-box_points->points[2].z;
  box_plane_normal2 << box_points->points[0].x-box_points->points[4].x,box_points->points[0].y-box_points->points[4].y,box_points->points[0].z-box_points->points[4].z;

  for(int i=0;i<cloud->size();i++){
    float x=cloud->points[i].x;
    float y=cloud->points[i].y;
    float z=cloud->points[i].z;

    int cout0=0;
    int cout1=0;
    int cout2=0;

    for(int j=0;j<box_points->size();j++){
      Eigen::Vector3d normal_tem;
      normal_tem << x-box_points->points[j].x,y-box_points->points[j].y,z-box_points->points[j].z;

      if(box_plane_normal0.dot(normal_tem)<0){
        cout0++;
      }

      if(box_plane_normal1.dot(normal_tem)<0){
        cout1++;
      }

      if(box_plane_normal2.dot(normal_tem)<0){
        cout2++;
      }
    }

    if(cout0==4&&cout1==4&&cout2==4){
      inliers->indices.push_back(i);
    }
  }
}

//VCCS over-segmentation
void VCCS_over_segmentation(PointCloudPtr_RGB_NORMAL cloud, float voxel_resolution,float seed_resolution,float color_importance,float spatial_importance,float normal_importance,vector<MyPointCloud_RGB_NORMAL>& patch_clouds, PointCloudT::Ptr colored_cloud, PointNCloudT::Ptr normal_cloud){
 
  PointCloudT::Ptr ct(new PointCloudT);
  NormalCloudTPtr normals(new NormalCloudT);

  for(int l=0;l<cloud->size();l++){
    PointT ptt;
    Normal normal;
    ptt.x=cloud->at(l).x;
    ptt.y=cloud->at(l).y;
    ptt.z=cloud->at(l).z;
    ptt.r=cloud->at(l).r;
    ptt.g=cloud->at(l).g;
    ptt.b=cloud->at(l).b;
    ptt.a=0;
    normal.normal_x=cloud->at(l).normal_x;
    normal.normal_y=cloud->at(l).normal_y;
    normal.normal_z=cloud->at(l).normal_z;
    ct->push_back(ptt);
    normals->push_back(normal);
  }

  pcl::SupervoxelClustering<PointT> super (voxel_resolution, seed_resolution, false);
  super.setInputCloud (ct);
  super.setNormalCloud (normals);
  super.setColorImportance (color_importance);
  super.setSpatialImportance (spatial_importance);
  super.setNormalImportance (normal_importance);

  std::map <uint32_t, pcl::Supervoxel<PointT>::Ptr > supervoxel_clusters;

  printf("Extracting supervoxels!\n");
  super.extract (supervoxel_clusters);
  printf("Found %d supervoxels\n", supervoxel_clusters.size ());

  vector<MyPointCloud_RGB_NORMAL> patch_clouds_tem;
  super.getPatchCloud(patch_clouds_tem);

  for(int i=0;i<patch_clouds_tem.size();i++){
    if(patch_clouds_tem.at(i).mypoints.size()>0){
      patch_clouds.push_back(patch_clouds_tem.at(i));
    }
  }

  PointCloudT::Ptr cvc = super.getColoredCloud();
  pcl::copyPointCloud(*cvc,*colored_cloud);

  PointNCloudT::Ptr sv_normal_cloud = super.makeSupervoxelNormalCloud (supervoxel_clusters);
  pcl::copyPointCloud(*sv_normal_cloud,*normal_cloud);

}