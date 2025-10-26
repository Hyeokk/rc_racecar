//--------------2D LiDAR lidar_lib------------------------|

#ifndef _LIDAR_LIB_H_
#define _LIDAR_LIB_H_

#include <ros/ros.h>
#include <iostream>
#include <cmath>

#include "asw_lidar/obs.h"

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include <std_msgs/Bool.h> // 추가

using namespace std;

typedef pcl::PointXYZ PointType;

bool cmp(const PointType &p1 ,const PointType &p2){
  //return (p1.x*p1.x + p1.y*p1.y) < (p2.x*p2.x + p2.y*p2.y);
  //return atan2(p1.y, p1.x) > atan2(p2.y, p2.x);
  
  if(p1.x < 0.5 && p2.x < 0.5) return p1.x < p2.x;
  else return p1.z < p2.z;
}

class Lidar_Func{
  protected:
    ros::NodeHandle nh_;
    ros::Publisher pub_roi_, pub_, pub_markers, pub_path_;
    string frame_id;
    int hz;
  
  public:
    //CLUSTERING
    void voxel(const pcl::PointCloud<PointType>::Ptr cloud_in, float voxel_size);
    void set_roi(const pcl::PointCloud<PointType>::Ptr cloud_in,  float roi[4]);
    void ransac(const pcl::PointCloud<PointType>::Ptr &cloud , const pcl::PointCloud<PointType>::Ptr &inliers, const pcl::PointCloud<PointType>::Ptr &outliers, double coeffi[], int modelNum, float radius);
    void clustering(const pcl::PointCloud<PointType>::Ptr cloud_in, vector<PointType> &center, float param[3]);
 
    void visualize_rviz(const vector<PointType> &obj, const string &frame_id, int type);
    void visualize_marker_geo(const geometry_msgs::PoseStamped *input_points, string id, int type);
    void visualize_markers(const pcl::PointCloud<PointType>::Ptr input_points, string id, int type);
    void visualize_path(vector<double>path_x_, vector<double>path_y_, string id);
    void rviz_clear(const string &frame_id);

    Lidar_Func(){
        ROS_INFO("Lidar_Func Start");
        ros::param::get("/parent/frame_id", frame_id);

        pub_roi_ = nh_.advertise<sensor_msgs::PointCloud2>("/roi_check", 1, true);
        pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/objects", 2, true);
        pub_markers = nh_.advertise<visualization_msgs::Marker>("/markers", 1, true);
        pub_path_ = nh_.advertise<nav_msgs::Path>("/path", 1 , true);
	}
    ~Lidar_Func(){
        ROS_INFO("Lidar_Func Terminated");
    }
};


void Lidar_Func::voxel(const pcl::PointCloud<PointType>::Ptr cloud_in, float voxel_size){
  pcl::VoxelGrid<PointType> vg;
  vg.setInputCloud (cloud_in);
  vg.setLeafSize (voxel_size, voxel_size, voxel_size);
  vg.filter (*cloud_in);
}

//roi[x_min, x_max, y_min, y_max]
void Lidar_Func::set_roi(const pcl::PointCloud<PointType>::Ptr cloud_in,  float roi[4]){
  pcl::PassThrough<PointType> pass; 

  pass.setInputCloud(cloud_in);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits(roi[0], roi[1]);
  pass.filter(*cloud_in);

  pass.setInputCloud(cloud_in);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(roi[2], roi[3]);
  pass.filter(*cloud_in);

  sensor_msgs::PointCloud2 to_rviz;
  pcl::toROSMsg(*cloud_in, to_rviz);
  to_rviz.header.frame_id = frame_id;
  to_rviz.header.stamp = ros::Time::now();
  pub_roi_.publish(to_rviz);
}

// modelNum: 0(plane), 1(line), 2(cylinder), 3(sphere), 4(cone) etc
void Lidar_Func::ransac(const pcl::PointCloud<PointType>::Ptr &cloud , const pcl::PointCloud<PointType>::Ptr &inliers, const pcl::PointCloud<PointType>::Ptr &outliers, double coeffi[6], int modelNum, float radius = -1) 
{
  // Object for Line fitting
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());

  pcl::SACSegmentation<PointType> seg;
  pcl::PointIndices::Ptr tmp(new pcl::PointIndices);
  seg.setOptimizeCoefficients (true);       //(옵션) // Enable model coefficient refinement (optional).
  seg.setInputCloud (cloud);                //입력 
  seg.setModelType (modelNum);              //적용 모델
  seg.setMethodType (pcl::SAC_RANSAC);      //적용 방법   // Use RANSAC method.
  seg.setMaxIterations (10000);              //최대 실행 수
  seg.setDistanceThreshold (0.1);           //inlier로 처리할 거리 정보   // Set the maximum allowed distance to the model.
  if (radius != -1) {                       //radius가 필요한 model인 경우, 
  seg.setRadiusLimits(0, radius);}             //Set minimum and maximum radii.
  seg.segment (*tmp, *coefficients);        //세그멘테이션 적용

  for (int i = 0; i < coefficients->values.size(); i++){
		coeffi[i] = coefficients->values[i];
	}

  pcl::copyPointCloud<PointType> (*cloud, *tmp, *inliers);

  pcl::ExtractIndices<PointType> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(tmp);
  extract.setNegative(true);
  extract.filter(*outliers);
}

// param[tol, min_size, max_size]
void Lidar_Func::clustering (const pcl::PointCloud<PointType>::Ptr cloud_in, vector<PointType> &center, float param[3]){

  pcl::search::KdTree<PointType>::Ptr kdtree(new pcl::search::KdTree<PointType>);
  kdtree -> setInputCloud(cloud_in);

  std::vector<pcl::PointIndices> clusterIndices;
  pcl::EuclideanClusterExtraction<PointType> ec;

  ec.setClusterTolerance(param[0]);
  ec.setMinClusterSize((int)param[1]);
  ec.setMaxClusterSize((int)param[2]);
  ec.setSearchMethod(kdtree);
  ec.setInputCloud(cloud_in);
  ec.extract(clusterIndices);

  for (vector<pcl::PointIndices>::const_iterator it = clusterIndices.begin(); it != clusterIndices.end(); ++it) {
      pcl::PointCloud<PointType>::Ptr cluster_cloud (new pcl::PointCloud<PointType>);
      for (vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
        cluster_cloud->points.emplace_back(cloud_in->points[*pit]);
      }
      PointType P;
      pcl::PointXYZ min_pt, max_pt;
      pcl::getMinMax3D (*cluster_cloud, min_pt, max_pt);
      P.x = (max_pt.x + min_pt.x)/2;
      P.y = (max_pt.y + min_pt.y)/2;
      P.z = sqrt(pow(P.x,2)+pow(P.y,2));
      center.emplace_back(P);
  }
  sort(center.begin(),center.end(),cmp);
}

void Lidar_Func::visualize_marker_geo(const geometry_msgs::PoseStamped *input_points, string id, int type){
	visualization_msgs::Marker points;
	geometry_msgs::Point point;

	points.header.frame_id = id;
	points.header.stamp = ros::Time::now();
	points.action = visualization_msgs::Marker::ADD;
	points.pose.orientation.w = 1.0;
	points.type = visualization_msgs::Marker::POINTS;
	points.scale.x = 0.2;
	points.scale.y = 0.2;
	points.color.a = 1.0;

  switch(type){
    case 0:
      points.color.r = 1.0f;
      points.ns = "red_markers_geometry";
      points.id = 100;
      break;

    case 1:
      points.color.g = 1.0f;
      points.ns = "green_markers_geometry";
      points.id = 110;
      break;

    case 2:
      points.ns = "blue_markers_geometry";
      points.color.b = 1.0f;
      points.id = 111;
      break;
    
    default:
      points.color.r = 1.0f;
      points.ns = "red_markers_geometry";
      points.id = 100;
      break;
  }

  point.x = (*input_points).pose.position.x;
  point.y = (*input_points).pose.position.y;
  point.z = 0;
  points.points.push_back(point);

	pub_markers.publish(points);
}

void Lidar_Func::visualize_markers(const pcl::PointCloud<PointType>::Ptr input_points, string id, int type){
	visualization_msgs::Marker points;
	geometry_msgs::Point point;

	points.header.frame_id = id;
	points.header.stamp = ros::Time::now();
	points.action = visualization_msgs::Marker::ADD;
	points.pose.orientation.w = 1.0;
	points.type = visualization_msgs::Marker::POINTS;
	points.scale.x = 0.5; 
	points.scale.y = 0.5;
	points.color.a = 1.0;

  switch(type){
    case 0:
      points.color.r = 1.0f;
      points.ns = "red_markers";
      points.id = 777;
      break;

    case 1:
      points.color.g = 1.0f;
      points.ns = "green_markers";
      points.id = 888;
      break;

    case 2:
      points.ns = "blue_markers";
      points.color.b = 1.0f;
      points.id = 999;
      break;
    
    default:
      points.color.r = 1.0f;
      points.ns = "red_markers";
      points.id = 777;
      break;
  }



	for (int i = 0; i < (*input_points).size(); i++) {
		point.x = (*input_points)[i].x;
		point.y = (*input_points)[i].y;
		point.z = 0;
		points.points.push_back(point);
	}

	pub_markers.publish(points);
}


void Lidar_Func::visualize_path(vector<double>path_x_, vector<double>path_y_, string id){
	nav_msgs::Path path_msg_;
	path_msg_.header.stamp = ros::Time::now();
	path_msg_.header.frame_id = id;
	for (size_t i = 0; i < path_x_.size(); i++)
	{
		geometry_msgs::PoseStamped pose;
		pose.pose.position.x = path_x_[i];
		pose.pose.position.y = path_y_[i];
		pose.pose.orientation.w = 1.0;
		path_msg_.poses.push_back(pose);
	}
	pub_path_.publish(path_msg_);
}

void Lidar_Func::visualize_rviz(const vector<PointType> &obj, const string &frame_id, int type){
  visualization_msgs::Marker obj_pt;
	obj_pt.header.frame_id = frame_id;
	obj_pt.header.stamp = ros::Time::now();
	obj_pt.pose.orientation.w = 1.0;
	obj_pt.type = visualization_msgs::Marker::SPHERE;
  obj_pt.action = visualization_msgs::Marker::ADD;
	obj_pt.color.a = 1.0;
  obj_pt.scale.x = 0.1;
  obj_pt.scale.y = 0.1;
  obj_pt.scale.z = 0.1;
  switch(type){
    case 0:
      obj_pt.ns ="centers";
      obj_pt.color.r = 1.0f;
      break;

    case 1:
      obj_pt.ns ="outline_cones";
      obj_pt.color.g = 1.0f;
      break;

    case 2:
      obj_pt.ns ="inline_cones";
      obj_pt.color.b = 1.0f;
      break;
    
    default:
      obj_pt.ns ="etc_centers";
      obj_pt.color.r = 1.0f;
      break;
  }
  visualization_msgs::MarkerArray ma;
  for (int i = 0 ; i<obj.size();i++){
      //create a center point
      obj_pt.id = i;
      obj_pt.pose.position.x = obj[i].x;
      obj_pt.pose.position.y = obj[i].y;
      obj_pt.pose.position.z = 0;
      ma.markers.emplace_back(obj_pt);
	}

  pub_.publish(ma);
}

void Lidar_Func::rviz_clear(const string &frame_id) {
  visualization_msgs::MarkerArray boxes;
  visualization_msgs::Marker node_name;
  node_name.header.frame_id = frame_id; // map frame 기준
  node_name.header.stamp = ros::Time::now();
  node_name.ns = "DEL"; 
  node_name.color.a = 1.0;
  node_name.scale.z = 1.0;
  node_name.action = visualization_msgs::Marker::DELETEALL;
  boxes.markers.emplace_back(node_name);
  pub_.publish(boxes);
}

#endif
