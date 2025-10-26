//------------Jeju island 3D Tunnel Task------------------------|
//------------MANDO 2D ROI--------------------------------------|

#include "lidar_lib/lidar_lib.h"
#include "lidar_lib/cubic_spline_planner.h"

bool Density_cmp(const PointType &p1, const PointType &p2){
  return p1.x < p2.x;
  //---------In MANDO Density---------|
  // if(p1.x < 1 && p2.x < 1){
  //   return p1.x < p2.x;
  // }
  // else{
  //   return p1.z < p2.z;
  // }
  //----------------------------------|
}

//l_cmp는 두 점의 각도를 비교하여 v1이 v2보다 각도상에서 더 뒤에 있으면 true를 반환합니다.
bool l_cmp(const PointType &v1, const PointType &v2){
  return atan2(v1.y, v1.x) > atan2(v2.y, v2.x);
}

//r_cmp는 두 점의 각도를 비교하여 v1이 v2보다 각도상에서 더 앞에 있으면 true를 반환합니다.
bool r_cmp(const PointType &v1, const PointType &v2){
  return atan2(v1.y, v1.x) < atan2(v2.y, v2.x);
}

class Task2 : public Lidar_Func{
private:
//-------------param-------------------------|
  float voxel_size;
  float roi[4];
  float current_speed_;
  float chan_dist_offset;
  float outcourse_offset;
  float ratio_line;
  float ratio_line2;
  float LD;
  float speed_total_;
  float corner_speed;
 
  int before_angle;
  int steer_total_;
  int target_index_;
  int straight_count_param;
  int straight_count;
  int ratio_line2_count;
  int done_count;
  int done_count_param;
  int straight_angle;

  double first_l;
  double first_r;
  double dist_offset;
  double cone_dist_;

  bool chan_flag;
  bool straight_flag;
  bool straight_flag2;

//------------MSG---------------------------|

  std_msgs::Int32 density_done_flag;

//------------vector------------------------|
  vector<PointType> center;
  vector<PointType> outline_cones;
  vector<PointType> inline_cones;
  vector<int> center_index;
  vector<double> r_rx, r_ry, r_ryaw, r_rk, l_rx, l_ry, l_ryaw, l_rk;
  vector<double> avg_vec_;

//------------visual waypoint---------------|
  geometry_msgs::Point wp;
  geometry_msgs::Twist cmd_vel_msg;


//------------PCL---------------------------|
  sensor_msgs::PointCloud::Ptr msg;
  pcl::PointCloud<PointType>::Ptr msg_;

//------------PUB---------------------------|
  ros::Publisher cmd_vel;
  ros::Publisher pub_path_l;
  ros::Publisher pub_path_r;
  ros::Publisher pub_flag;

//------------SUB---------------------------|
  ros::Subscriber sub_points;

public:
  Task2(){
    initsetup();
    msg.reset(new sensor_msgs::PointCloud());
    msg_.reset(new pcl::PointCloud<pcl::PointXYZ>());
  }
  ~Task2(){
    ROS_INFO("Task2 Done");
  }

  void initsetup();
  void run();
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan);
  void vector_clear();
  void decision();
  void make_line();
  void make_waypoint();
  void vector_from_pcl(const pcl::PointCloud<PointType>::Ptr cloud_in);
  bool is_in_vector(vector<int> v, int element);
  int GetAverage(vector<double> const& vec);
  int pure_pursuit(vector<double>& rx, vector<double>& ry, geometry_msgs::PoseStamped& target_p, float& speed, vector<double>& rk);
};

void Task2::scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan){
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

void Task2::initsetup(){
  //------------pre-process---------------------------|
  ros::param::get("/Task2/min_x_", roi[0]);
  ros::param::get("/Task2/max_x_", roi[1]);
  ros::param::get("/Task2/min_y_", roi[2]);
  ros::param::get("/Task2/max_y_", roi[3]);
  ros::param::get("/Task2/voxel_size", voxel_size);

  //------------Task2 process-------------------------|
  ros::param::get("/Task2/cone_dist_", cone_dist_);
  ros::param::get("/Task2/dist_offset", dist_offset);
  ros::param::get("/Task2/chan_dist_offset", chan_dist_offset); // 직각만 보일 때(left와 right가 직각으로 잡힌 경우), 패스 생성 offset
  ros::param::get("/Task2/outcourse_offset", outcourse_offset); // 들어가기 전에 아웃 코스를 타기 위한 offset
  ros::param::get("/Task2/straight_count_param", straight_count_param); // 나갈 때 직진을 하기 위한 곡선 카운트
  ros::param::get("/Task2/ratio_line", ratio_line); // out line과 in line의 비율 (얼마나 더 빨리 최대 조향각으로 틀 지 결정)
  ros::param::get("/Task2/ratio_line2", ratio_line2);
  ros::param::get("/Task2/done_count_param", done_count_param); // done flag count
  ros::param::get("/Task2/corner_speed", corner_speed);
  ros::param::get("/Task2/LD", LD);
  ros::param::get("/Task2/straight_angle", straight_angle);

  //-------------PUB------------------------------------|
  cmd_vel = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  pub_flag = nh_.advertise<std_msgs::Int32>("mission_lidar",1);
  pub_path_l = nh_.advertise<nav_msgs::Path>("left_path",1);
  pub_path_r = nh_.advertise<nav_msgs::Path>("right_path",1);

  //-------------SUB-------------------------------------|
  sub_points = nh_.subscribe("/scan", 1, &Task2::scanCallback, this);

  //-------------index 초기화-----------------------------|
  first_l = 0.2;
  first_r = -0.2;
  
  before_angle = 0; 
  
  current_speed_ = 0.0f; 
  
  target_index_ = 0;

  chan_flag = false;

  straight_flag = false;
  straight_flag2 = false;
  straight_count = 0;
  
  done_count = 0;
  
  ratio_line2_count = 0;

  density_done_flag.data = -1;

  r_rx.resize(1);
  r_ry.resize(1);
  r_ryaw.resize(1);
  r_rk.resize(1);

  l_rx.resize(1);
  l_ry.resize(1);
  l_ryaw.resize(1);
  l_rk.resize(1);
}

void Task2::vector_clear(){
    center.clear();
    outline_cones.clear();
    inline_cones.clear();
    center_index.clear();

    r_rx.clear();
    r_ry.clear();
    r_ryaw.clear();
    r_rk.clear();

    l_rx.clear();
    l_ry.clear();
    l_ryaw.clear();
    l_rk.clear();
}

void Task2::decision(){

    vector<vector<double>> cloud_l, cloud_r;
    vector<double> x, y, rx, ry, ryaw, rk; 

    int first_left_index = 0, first_right_index = 0;
    double yy;
    bool left_flag = false, right_flag = false;

    for(int i = 0; i < center.size();i++){
      center_index.push_back(i);
    }

    for (size_t i = 0; i < center_index.size(); i++)
    {
      PointType point;

      yy = center[center_index[i]].y;

      if(yy > 0 && !left_flag){
        left_flag = true;
        first_left_index = i;
      }
      else if (yy < 0 && !right_flag){
        right_flag = true;
        first_right_index = i;
      }
      if(left_flag && right_flag){
        break;
      }
    }
  
    vector<int> left_index;
    left_index.push_back(first_left_index);
    for(int i = 0; i < center_index.size(); i++){
      if(is_in_vector(left_index,i)){
        continue;
      }
      double temp_x = center[left_index.back()].x;
      double temp_y = center[left_index.back()].y;
      int index = 0;
      double min_dist = 9999;
      for(int j = 0; j < center_index.size(); j++){
        double now_dist = sqrt(pow(temp_x - center[center_index[j]].x,2) + pow(temp_y - center[center_index[j]].y,2));
        if(min_dist > now_dist && !is_in_vector(left_index,j)){
          min_dist = now_dist;
          index = j;
        }
      }
      if(sqrt(pow(temp_x - center[center_index[index]].x,2) + pow(temp_y - center[center_index[index]].y,2)) > cone_dist_){
        break;
      }
      else{
        left_index.push_back(index);
      }
    }

    vector<int> right_index;
    right_index.push_back(first_right_index);
    for(int i = 0; i < center_index.size(); i++){
      if(is_in_vector(right_index,i)){
        continue;
      }
      for(int k=0; k < right_index.size(); k++){
        double temp_x = center[right_index[k]].x;
        double temp_y = center[right_index[k]].y;
        double min_dist = 9999;
        int index = -1;
        for(int j = 0; j < center_index.size(); j++){
          double now_dist = sqrt(pow(temp_x - center[center_index[j]].x,2) + pow(temp_y - center[center_index[j]].y,2));
          if(min_dist > now_dist && !is_in_vector(right_index,j)){
            min_dist = now_dist;
            index = j;
          }
        }
        if (index == -1) {
            break;
        }
        if(sqrt(pow(temp_x - center[center_index[index]].x,2) + pow(temp_y - center[center_index[index]].y,2)) > cone_dist_){
          break;
        }
        else {
          right_index.push_back(index);
        }
      }
    }
    vector<int> duplicates;
    for (int value : left_index) {
        if (std::find(right_index.begin(), right_index.end(), value) != right_index.end()) {
            duplicates.push_back(value);
        }
    }

    // 중복된 값 제거
    for (int value : duplicates) {
        right_index.erase(std::remove(right_index.begin(), right_index.end(), value), right_index.end());
    }


  cout << "left index        : " << left_index.size() << endl;
  cout << "right index       : " << right_index.size() << endl;
  cout << "center index size : " << center_index.size() << endl;
  cout << "center size       : " << center.size() << endl;
  if(center.size() > 0){
    for(int i = 0; i < left_index.size();i++){
      outline_cones.push_back(center[left_index[i]]);
    }
    for(int i = 0; i < right_index.size();i++){
      inline_cones.push_back(center[right_index[i]]);
    }
  }

    // 교점 찾기
  float points_min_dist = 10.00;
  for(int i=0; i<outline_cones.size(); i++){
    for(int j=0; j<inline_cones.size(); j++){
      float temp = sqrt(pow(outline_cones[i].x - inline_cones[j].x,2) + pow(outline_cones[i].y - inline_cones[j].y,2));
      if(points_min_dist > temp){
        points_min_dist = temp;
      }
    }
  } 

  if(points_min_dist < 0.4){
    for(int i = 0; i < outline_cones.size();i++){
      inline_cones.push_back(outline_cones[i]);
    }
    outline_cones. clear();
    chan_flag = true;
  }

  sort(outline_cones.begin(),outline_cones.end(),l_cmp);
  sort(inline_cones.begin(),inline_cones.end(),r_cmp);
}

void Task2::make_line() {
  vector<double> input_x_l, input_y_l,input_x_r,input_y_r;
  input_x_l.clear();
  input_y_l.clear();
  input_x_r.clear();
  input_y_r.clear();
  double ds = 0.01;

  if(outline_cones.size() > 0){
      input_x_l = {-0.2};
      input_y_l = {first_l};
  }
  if(inline_cones.size() > 0){
      input_x_r = {-0.2};
      input_y_r = {first_r};
  }


  for (size_t i = 0; i < outline_cones.size(); i++) 
  {
    double x,y;

    x = outline_cones[i].x;
    if(!straight_flag) y = outline_cones[i].y - outcourse_offset;
    else if(straight_flag) y = outline_cones[i].y;
    input_x_l.push_back(x);
    input_y_l.push_back(y);

  }

  for (size_t i = 0; i < inline_cones.size(); i++) 
  {
    double x,y;
    x = inline_cones[i].x;
    y = inline_cones[i].y;
    
    input_x_r.push_back(x);
    input_y_r.push_back(y);
  }


  if (input_x_l.size() < 3){//예외처리 코드
    input_x_l.clear();
    input_y_l.clear();
    double before_dx, before_dy;
    if(input_x_r.size() >=3){
      for(int i = 0; i < input_x_r.size(); i++){
        if(i < input_x_r.size()-1){
          double dx = input_x_r[i+1] - input_x_r[i];
          before_dx = dx;
          double dy = input_y_r[i+1] - input_y_r[i];
          before_dy = dy;
          
          double m = -1 * dx/dy;
          if((dx > 0 && dy > 0) || (dx < 0 && dy > 0)){
            input_x_l.push_back(input_x_r[i] - (dist_offset) / (sqrt(m*m + 1)));
            input_y_l.push_back(input_y_r[i] - (dist_offset)*m / (sqrt(m*m + 1)));
          }
          else{
            input_x_l.push_back(input_x_r[i] + (dist_offset) / (sqrt(m*m + 1)));
            input_y_l.push_back(input_y_r[i] + (dist_offset)*m / (sqrt(m*m + 1)));
          }
        }
        else{
          double dx = before_dx;
          double dy = before_dy;
          
          double m = -1 * dx/dy;
          if((dx > 0 && dy > 0) || (dx < 0 && dy > 0)){
            input_x_l.push_back(input_x_r[i] - (dist_offset) / (sqrt(m*m + 1)));
            input_y_l.push_back(input_y_r[i] - (dist_offset)*m / (sqrt(m*m + 1)));
          }
          else{
            input_x_l.push_back(input_x_r[i] + (dist_offset) / (sqrt(m*m + 1)));
            input_y_l.push_back(input_y_r[i] + (dist_offset)*m / (sqrt(m*m + 1)));
          }
 
        }
      }
    }
    else{
      input_x_l.resize(2);
      input_y_l.resize(2);
      input_x_l = {0.0,0.3};
      input_y_l = {0.2,0.2};
    }
    calc_spline_course(input_x_l,input_y_l,l_rx,l_ry,l_ryaw,l_rk,ds);
  }
  else{
    calc_spline_course(input_x_l,input_y_l,l_rx,l_ry,l_ryaw,l_rk,ds);
  }  

  if (input_x_r.size() < 3){//예외처리 코드
    input_x_r.clear();
    input_y_r.clear();
    double before_dx,before_dy;
    if(input_x_l.size() >=3){
      for(int i = 0; i < input_x_l.size(); i++){
        if(i < input_x_l.size() - 1){
          double dx = input_x_l[i+1] - input_x_l[i];before_dx = dx;
          double dy = input_y_l[i+1] - input_y_l[i];before_dy = dy;
          
          double m = -1 * dx/dy;

          if((dx > 0 && dy < 0) || (dx < 0 && dy < 0)){
          input_x_r.push_back(input_x_l[i] - (dist_offset) / (sqrt(m*m + 1)));
          input_y_r.push_back(input_y_l[i] - (dist_offset)*m / (sqrt(m*m + 1)));
          }
          else{
            input_x_r.push_back(input_x_l[i] + (dist_offset) / (sqrt(m*m + 1)));
            input_y_r.push_back(input_y_l[i] + (dist_offset)*m / (sqrt(m*m + 1)));
          }
        }
        else{
          double dx = before_dx;
          double dy = before_dy;
          
          double m = -1 * dx/dy;

          if((dx > 0 && dy < 0) || (dx < 0 && dy < 0)){
          input_x_r.push_back(input_x_l[i] - (dist_offset) / (sqrt(m*m + 1)));
          input_y_r.push_back(input_y_l[i] - (dist_offset)*m / (sqrt(m*m + 1)));
          }
          else{
            input_x_r.push_back(input_x_l[i] + (dist_offset) / (sqrt(m*m + 1)));
            input_y_r.push_back(input_y_l[i] + (dist_offset)*m / (sqrt(m*m + 1)));
          }

        }
      }
    }
    else{
      input_x_r.resize(2);
      input_y_r.resize(2);
      input_x_r = {0.0,0.3};
      input_y_r = {-0.2,-0.2};
    }
    calc_spline_course(input_x_r,input_y_r,r_rx,r_ry,r_ryaw,r_rk,ds);
  }
  else{
    calc_spline_course(input_x_r,input_y_r,r_rx,r_ry,r_ryaw,r_rk,ds);
  }

  nav_msgs::Path path_msg_l;
  nav_msgs::Path path_msg_r;

  path_msg_l.header.stamp = ros::Time::now();
  path_msg_l.header.frame_id = frame_id;
  for (size_t i = 0; i < l_rx.size(); i++)
  {
      geometry_msgs::PoseStamped pose;
      pose.pose.position.x = l_rx[i];
      pose.pose.position.y = l_ry[i];
      pose.pose.orientation.w = 1.0;
      path_msg_l.poses.push_back(pose);
  }
  pub_path_l.publish(path_msg_l);

  path_msg_r.header.stamp = ros::Time::now();
  path_msg_r.header.frame_id = frame_id;
  for (size_t i = 0; i < r_rx.size(); i++)
  {
      geometry_msgs::PoseStamped pose;
      pose.pose.position.x = r_rx[i];
      pose.pose.position.y = r_ry[i];
      pose.pose.orientation.w = 1.0;
      path_msg_r.poses.push_back(pose);
  }
  pub_path_r.publish(path_msg_r);


  if (r_ry.size() >= 3){
    first_r = 0;
    first_r += (r_ry[0] + r_ry[1] + r_ry[2]);
    first_r /= 3;
  }
  if (l_rx.size() >= 3){
    first_l = 0;
    first_l += (l_ry[0] + l_ry[1] + l_ry[2]);
    first_l /= 3;
  }
}

bool Task2::is_in_vector(vector<int> v, int element){
    vector<int>::iterator it;
    it = find(v.begin(), v.end(), element);
    if (it != v.end()) {
        return true;
    } else {
        return false;
    }
}

void Task2::make_waypoint(){    //size = (int)(rx.size()*0.5);
  vector<double> input_x, input_y,rx,ry,ryaw,rk;
  double ds = 0.01;
  float angle = 0.0f;
  float speed = 0.0f;
  float off_set_x = 0.1;
  float off_set_y = 0.2;
  int size_ = (l_rx.size() >= r_rx.size()) ? r_rx.size() : l_rx.size();
  int count = 0;
  double sum_x = 0, sum_y = 0;
  int size;

  //중앙경로 생성
  for (int i = 0; i < size_; i++){
    if(count%3 == 0){
      sum_x += (l_rx[i] + r_rx[i])/2 ;
      sum_y += (l_ry[i] + r_ry[i])/2 ;
      // 3 << "sum_x : : " << sum_x << endl;
      input_x.push_back(sum_x/3);
      input_y.push_back(sum_y/3);
      sum_x = 0;
      sum_y = 0;
    }
    else{
      sum_x += (l_rx[i] + r_rx[i])/2;
      sum_y += (l_ry[i] + r_ry[i])/2;
    }
    count++;
  }

  if(inline_cones.size() > 0 && outline_cones.size() >0){
    input_x.push_back((l_rx[l_rx.size()-1] + r_rx[r_rx.size()-1])/2);
    input_y.push_back((l_ry[l_ry.size()-1] + r_ry[r_ry.size()-1])/2);
  }

  input_x.erase(input_x.begin());
  input_y.erase(input_y.begin());

  if (input_x.size() < 2){//예외처리 코드
    input_x = {0.0,0.3};
    input_y = {0.0,0.0};
    calc_spline_course(input_x,input_y,rx,ry,ryaw,rk,ds);
  }
  else{
    calc_spline_course(input_x,input_y,rx,ry,ryaw,rk,ds);
  }

  if(chan_flag){
    input_x.clear();
    input_y.clear();
    input_x = {0.0,inline_cones[inline_cones.size()-1].x - chan_dist_offset};
    input_y = {0.0,inline_cones[inline_cones.size()-1].y};
    calc_spline_course(input_x,input_y,rx,ry,ryaw,rk,ds);
    chan_flag = false;
  }

  visualize_path(rx, ry, frame_id);

  size = (int)(rx.size()*0.5);

// --------------------------주행-------------------------------------------------------------------------//
  //cout << "waypoint size" << rx.size() << endl;
  geometry_msgs::PoseStamped target_p;
  if (rx.size() > 20)
  {
  //------------define angle---------
    //angle = pure_pursuit(rx, ry, target_p,speed);
    // angle = stanley(rx,ry,ryaw,rk);
    angle = pure_pursuit(rx, ry, target_p,speed,rk);

    before_angle = GetAverage(avg_vec_);

    wp.x = target_p.pose.position.x;
    wp.y = target_p.pose.position.y;
  }
  else{
    // cout << "방어 steer" << endl;
    angle = before_angle;
  }

  if(sqrt(pow(target_p.pose.position.x,2)+pow(target_p.pose.position.y,2))<0.2){
    angle = before_angle;
  }

  //4.범위 이외시 최대조향각
  if (angle >= 20)
    angle = 20;
  if (angle <= -15)
    angle = -35;

  //speed = return_speed(angle,rx,ry,rk); 
  // speed_avg_vec_.push_back(speed);
  // if (speed_avg_vec_.size() > 5){speed_avg_vec_.erase(speed_avg_vec_.begin());}
  // speed_total_ = GetAverage(speed_avg_vec_);

  speed_total_ = 0.35;

  steer_total_ = angle;

  if(straight_flag && (outline_cones.size()+ inline_cones.size()) != 0 && float(outline_cones.size())/float(outline_cones.size()+ inline_cones.size()) < ratio_line) { 
    steer_total_ = -30;
}
  
// -----------flag 추가--------------
  if(!straight_flag2 && straight_flag && (outline_cones.size()+ inline_cones.size()) != 0 && float(outline_cones.size())/float(outline_cones.size()+ inline_cones.size()) > ratio_line2) {
    ratio_line2_count++;
    if(ratio_line2_count == 10) straight_flag2 = true;
}
  else if(!straight_flag2 && straight_flag && (outline_cones.size()+ inline_cones.size()) != 0 && float(outline_cones.size())/float(outline_cones.size()+ inline_cones.size()) <= ratio_line2) {
   ratio_line2_count = 0;
}
//----------------------------------

  if(steer_total_ < -10 && !straight_flag2) speed_total_ = corner_speed;

// ------------기존 flag---------------
  if (steer_total_ <= straight_angle){
    straight_count++;
    if(straight_count == straight_count_param) {
      straight_flag = true;
    }
  }
  else straight_count = 0;

  if (straight_flag && inline_cones.size() <= 20){
    done_count++;
    if(done_count == done_count_param){
    density_done_flag.data = 5;
    pub_flag.publish(density_done_flag);
    }
  }
  else done_count = 0;

// -------------------------------

  cmd_vel_msg.linear.x = speed_total_;
  cmd_vel_msg.angular.z = steer_total_;

  cmd_vel.publish(cmd_vel_msg);

  cout << "speed_total_  = " << speed_total_ << endl; 
  cout << "steer_total_  = " << steer_total_ << endl;
  cout << "직진 플래그 = " << straight_flag << endl;
  cout << "done 플래그 = "<< density_done_flag.data <<endl;
  cout << "\n" <<endl;

  avg_vec_.push_back(angle);
  
  if(avg_vec_.size() > 10){ avg_vec_.erase(avg_vec_.begin());}

}

int Task2::GetAverage(vector<double> const& vec){
    if (vec.empty()){
        return 0;
    }
    return accumulate(vec.begin(),vec.end(),0.0) / vec.size();
}

int Task2::pure_pursuit(vector<double>& rx, vector<double>& ry, geometry_msgs::PoseStamped& target_p,float& speed,vector<double>& rk)
{
  // 1. 목표 추종점 찾기
  //float lookahead_dist = speed* 0.2 + 0.5; // temp_speed = m/s
  // float lookahead_dist = 5.5;

  // #### LD 피팅 ####
// 1안
    // float lookahead_dist = 0.03*(0.03*current_speed_*current_speed_*current_speed_+7.5*current_speed_)+2;
// 2안
    // lookahead_dist_ = tanf(0.08*cur_speed_ - 0.09)+4.0;
    float lookahead_dist = LD;

    geometry_msgs::PoseStamped local_pose;
    local_pose.pose.position.x = 0.0;
    local_pose.pose.position.y = 0.0;    
    
    //int min_idx = find_cloestindex(local_pose,rx,ry);
    // double ratio_gain = 0;
    // if (rk.size() > 11) ratio_gain =abs(rk[10]);
    // else ratio_gain = abs(rk.back());
    // if(ratio_gain > 0.2) lookahead_dist = 4.5;

  float final_ld, alpha;
  float cur_x = -0.18f; // 휠베이스
  float cur_y = 0.0f;
  float dist = 0.0f;

  if(straight_flag2){
    for (int i = 0; i < rx.size(); i++)
    {
      dist = sqrt(pow(rx[i]-cur_x,2) + pow(ry[i]-cur_y,2));
      if (dist > lookahead_dist)
      {
        target_index_ = i;
        final_ld = dist; // 최종으로 사용할 ld
        break;
      }
      if (i == rx.size())
      {
        // cout << "30개 이상일 때 방어 " << endl;
        target_index_ = i;
        final_ld = dist;
        break;
      }
    }
    target_p.pose.position.x = rx[target_index_];
    target_p.pose.position.y = ry[target_index_];
  }
  else if(70 < rx.size() && rx.size() < 200 && straight_flag) // 위에 Ld랑 맞추기
  {
    target_p.pose.position.x = rx[rx.size()-1];
    target_p.pose.position.y = ry[ry.size()-1];
    target_index_ = rx.size()-1;
    final_ld = sqrt(pow(target_p.pose.position.x-cur_x,2)+pow(target_p.pose.position.y-cur_y,2));
  }
  else
  {
    for (int i = 0; i < rx.size(); i++)
    {
      dist = sqrt(pow(rx[i]-cur_x,2) + pow(ry[i]-cur_y,2));
      if (dist > lookahead_dist)
      {
        target_index_ = i;
        final_ld = dist; // 최종으로 사용할 ld
        break;
      }
      if (i == rx.size())
      {
        // cout << "30개 이상일 때 방어 " << endl;
        target_index_ = i;
        final_ld = dist;
        break;
      }
    }
    target_p.pose.position.x = rx[target_index_];
    target_p.pose.position.y = ry[target_index_];
  }
  visualize_marker_geo(&target_p, frame_id, 1);
  double x = target_p.pose.position.x + abs(cur_x);
  double y = target_p.pose.position.y;

  // 2. alpha 구하기 
  alpha = -1*atan2f(y,x); // 일반적으로 아는 축으로 맞춤
  // if(alpha < -90.0 && alpha > -180.0f) alpha += 270.0f;   //지우는게 맞는듯...?
  // else alpha -= 90.0f;
  // alpha = -alpha*M_PI/180.0f;
  
  //3. 최종 조향각
  if (final_ld > 1.3f)
    final_ld = 1.3f;
  if (final_ld < 0.5f)
    final_ld = 0.5f;

  float cur_steer = atan2f(2.0f * 0.18f * sinf(alpha) / (final_ld), 1.0f) * -180.0f / M_PI ; 
  cur_steer *= 0.2;
  // if(!straight_flag && cur_steer<0) cur_steer *= 2.5;
  // else if(straight_flag && cur_steer<0) cur_steer *= 3.0;
  // else if(cur_steer>0) cur_steer *= 2.5;
  
  // if (cur_steer > 14.25f) {
  //   cur_steer = 14.25f;  // 0.3rad 이상일 때
  // } 
  // else if (cur_steer < -14.18f) {
  //   cur_steer = -14.18f; // -0.3rad 이하일 때
  // }

  cout << "===== Pure-pursuit ====="<< endl;
  cout << "target_index_" << target_index_<<endl;
  cout << "lookahead distance   : " << lookahead_dist <<endl;
  cout << "final ld             : " << final_ld <<endl;
  cout << "pure pursuit steer   : " << cur_steer << endl;
  cout << "steer: "<< cmd_vel_msg.angular.z<<endl;
  cout << "dist: " << sqrt(pow(target_p.pose.position.x,2)+pow(target_p.pose.position.y,2)) <<endl;
  cout << "rx.size(): "<<rx.size()<<endl;
  return cur_steer;

}

void Task2::vector_from_pcl(const pcl::PointCloud<PointType>::Ptr cloud_in){
  for(int i = 0; i < cloud_in->size(); i++){
    PointType P;
    P.x = (*cloud_in)[i].x;
    P.y = (*cloud_in)[i].y;
    P.z = sqrt(pow((*cloud_in)[i].x,2)+((*cloud_in)[i].y,2));
    center.emplace_back(P);
  }
  sort(center.begin(), center.end(), cmp);
}

void Task2::run(){
  if(!straight_flag) roi[3] = 0.6;
  else if(straight_flag) roi[3] = 1.5;
  if(density_done_flag.data != 5){
    vector_clear();
    set_roi(msg_, roi);
    voxel(msg_, voxel_size);
    vector_from_pcl(msg_);
    decision();
    //visualize_rviz(center, frame_id , 0);
    rviz_clear(frame_id);
    visualize_rviz(outline_cones, frame_id , 1);
    visualize_rviz(inline_cones, frame_id , 2);
    make_line();
    make_waypoint();
  }
}