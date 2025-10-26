//정적 장애물 회피 맨 왼쪽차선에서 진행
#include "lidar_lib/lidar_lib.h"
#include "lidar_lib/cubic_spline_planner.h"


// bool static_cmp(const PointType &p1, const PointType &p2){
//   return p1.x < p2.x;
// }

// class Task5 : public Lidar_Func{
// private:
// //-------------Parameter----------------------------|
//   float voxel_size;
//   float roi[4];
//   float param[3];

//   int camera_static_flag;
//   int camera_state;
//   int dynamic_count;

//   std_msgs::Int32 Speed_Int32;
//   std_msgs::Int32 lidar_static_flag_msg;
//   std_msgs::Int32 lidar_dynamic_flag_msg;
//   std_msgs::Float32 SteerAngle_Float32;

// //-------------Vector--------------------------------|
//   vector<PointType> center;

// //-------------PCL-----------------------------------|
//   pcl::PointCloud<PointType>::Ptr msg_
//   sensor_msgs::PointCloud::Ptr msg;

// //-------------PUB-----------------------------------|
  
// }


//-------ROI: 내 차선, 내 전방 15m----------------------|
class Task5 : public Lidar_Func{
private:
// param
    float voxel_size;
    float roi[4];
    float param[3];

//------------MSG---------------------------|
    std_msgs::Bool obstacle_flag;
//------------vector------------------------|
    vector<PointType> center_;

//------------PCL---------------------------|
    sensor_msgs::PointCloud::Ptr msg;
    pcl::PointCloud<PointType>::Ptr msg_;

//------------PUB---------------------------|
    ros::Publisher cmd_vel;
    ros::Publisher pub_flag;
    ros::Publisher pub_points_;
    ros::Publisher pub_center_;
    ros::Publisher pub_marker_;
    ros::Publisher pub_status_flag_;
//------------SUB---------------------------|
    ros::Subscriber sub_points;
    
public:
    Task5(){
        initsetup();
        msg.reset(new sensor_msgs::PointCloud());
        msg_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    }
    ~Task5(){
      ROS_INFO("Task5 Done");
    }
    void initsetup();
    void run();
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan);
    void vector_clear();
    void decision();
};


void Task5::scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan){
  laser_geometry::LaserProjection projector_;
  projector_.projectLaser(*scan, *msg);
  (*msg_).width = (*msg).points.size();
  (*msg_).height = 1;
  (*msg_).points.resize((*msg_).width * (*msg_).height);
  
  //전후방이 바뀐 경우 x, y에 - 부호 추가
  for(int i = 0; i < (*msg).points.size(); i++){
    (*msg_).points[i].x = (*msg).points[i].x;
    (*msg_).points[i].y = (*msg).points[i].y;
    (*msg_).points[i].z = 0; //2D LiDAR
  }
}


void Task5::initsetup(){
   ros::param::get("/Status_5/min_x_",  roi[0]);
    ros::param::get("/Status_5/max_x_",  roi[1]);
    ros::param::get("/Status_5/min_y_",  roi[2]);
    ros::param::get("/Status_5/max_y_",  roi[3]);

    ros::param::get("/Status_5/tolerance",  param[0]);
    ros::param::get("/Status_5/cluster_size_min",  param[1]);
    ros::param::get("/Status_5/cluster_size_max",  param[2]);
    ros::param::get("/Status_5/voxel_size", voxel_size);
    //------------pub----------------------------------------------------------------|
    //cmd_vel = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    //pub_flag = nh_.advertise<std_msgs::Int32>("mission_lidar",1);
    pub_status_flag_ = nh_.advertise<std_msgs::Bool>("/static_flag_topic", 1); 
    
    pub_points_ = nh_.advertise<sensor_msgs::PointCloud2>("passed_points", 10);
    pub_center_ = nh_.advertise<visualization_msgs::Marker>("center_points", 10);
    pub_marker_ = nh_.advertise<visualization_msgs::Marker>("wayPoint", 10);

    //------------sub----------------------------------------------------------------|
    sub_points = nh_.subscribe("/scan", 1, &Task5::scanCallback, this);
}
void Task5::vector_clear(){
    center_.clear();
}
void Task5::decision(){
    //안전거리 전방 10m(morai 7m) LiDAR센서 거리 전방 18m(morai 15m)
    float moving_distance = 20.0;
    
    cout << "count: " << center_.size() << endl;
    //----------------센터점이 존재하는 경우---------------------|
    if(!center_.empty()){
        float obstacle_distance=sqrt(pow(center_[0].x,2) + pow(center_[0].y,2));
        cout << "----------장애물 인식--------------" << endl;
        //-------------장애물거리와 안전거리가 같을 때------------|
        if(obstacle_distance <= moving_distance){
            cout << "---------안전거리 안에 장애물---------------" << endl;
            cout << "---------회 피 시 작--------------------" << endl;
            obstacle_flag.data = true;
            pub_status_flag_.publish(obstacle_flag);
        }
        else { //장애물 인식 안될때
            cout << " GOOOOOOOOOOOOOOOOOOOOOOOO" << endl;

            obstacle_flag.data = false;
            pub_status_flag_.publish(obstacle_flag);
        }
    }
    //--------------센터점이 존재하지 않을 때--------------|
    else{
        cout << "GOOOOOOOOOOOOOOOOOOOOOOOOOOOOO" << endl;
        obstacle_flag.data = false;
        pub_status_flag_.publish(obstacle_flag);
    }
}
void Task5::run(){
    vector_clear();
    set_roi(msg_, roi);
    voxel(msg_, voxel_size);
    clustering(msg_,center_, param);
    decision();

}