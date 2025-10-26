//------------Jeju island 3D cone Task------------------------|
//------------MANDO 2D ROI------------------------------------|

#include "lidar_lib/lidar_lib.h"
#include "lidar_lib/cubic_spline_planner.h"
#include <ackermann_msgs/AckermannDriveStamped.h>

bool track_cmp(const PointType &p1, const PointType &p2){
  return p1.x < p2.x;
}
bool duplicates_track_cmp(int a, int b){
  return a < b;
}

class Task1 : public Lidar_Func{
private:
//-------------param-------------------------|
  float voxel_size;
  float roi[4];
  float param[3];
  float speed_total_;
  float LD;
  double first_l, first_r;
  double cone_dist_;
  double dist_offset_;
  int before_angle;
  int steer_total_;
  int current_speed_;
  int target_index_;

//--------------vector-----------------------|
  vector<PointType> center;
  vector<PointType> outline_cones;
  vector<PointType> inline_cones;
  vector<int> center_index;
  vector<double> r_rx, r_ry, r_ryaw, r_rk, l_rx, l_ry, l_ryaw, l_rk;
  vector<double> avg_vec_;

//-----------visual waypoint-----------------|
  geometry_msgs::Point wp;
  ackermann_msgs::AckermannDriveStamped cmd_vel_msg;

//-------------PCL---------------------------|
  pcl::PointCloud<PointType>::Ptr msg_;
  sensor_msgs::PointCloud::Ptr msg;
//-------------PUB---------------------------|

  ros::Publisher pub_path_l;
  ros::Publisher pub_path_r;
  ros::Publisher cmd_vel;

//-------------SUB---------------------------|
  ros::Subscriber sub_points;

public:
    Task1(){
      initsetup();
      msg.reset(new sensor_msgs::PointCloud());
      msg_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    }
    ~Task1(){
      ROS_INFO("Task1 Done");
    }

    void initsetup();
    void run();
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan);
    void vector_clear();
    void decision();
    void make_line();
    bool is_in_vector(vector<int> v, int element);
    void make_waypoint();
    int GetAverage(vector<double> const& vec);
    int pure_pursuit(
      vector<double>& rx,
      vector<double>& ry,
      geometry_msgs::PoseStamped& target_p,
      float& speed,
      vector<double>& rk
    );
};

void Task1::scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan){
    laser_geometry::LaserProjection projector_;
    projector_.projectLaser(*scan, *msg);
    (*msg_).width = (*msg).points.size();
    (*msg_).height = 1;
    (*msg_).points.resize((*msg_).width * (*msg_).height);
    // 앞 뒤 바뀌면 x, y에 - add
  for (int i = 0; i < (*msg).points.size(); i++)
  {
    (*msg_).points[i].x = (*msg).points[i].x;
    (*msg_).points[i].y = (*msg).points[i].y;
    (*msg_).points[i].z = 0;
  }
}
//-----------------DATA 전처리-----------------------|
void Task1::initsetup(){
  ros::param::get("/Task1/min_x_", roi[0]);
  ros::param::get("/Task1/max_x_", roi[1]);
  ros::param::get("/Task1/min_y_", roi[2]);
  ros::param::get("/Task1/max_y_", roi[3]);
  ros::param::get("/Task1/voxel_size", voxel_size);
  ros::param::get("/Task1/tolerance", param[0]);
  ros::param::get("/Task1/cluster_size_min", param[1]);
  ros::param::get("/Task1/cluster_size_max", param[2]);
  ros::param::get("/Task1/cone_dist", cone_dist_);
  ros::param::get("/Task1/dist_offset", dist_offset_);
  ros::param::get("/Task1/LD", LD);

//--------------PUB--------------------------------------|
  cmd_vel = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("/cmd_ackermann/auto", 10);

  pub_path_l = nh_.advertise<nav_msgs::Path>("left_path", 1);
  pub_path_r = nh_.advertise<nav_msgs::Path>("right_path", 1);

//--------------SUB---------------------------------------|
  sub_points = nh_.subscribe("/scan", 1, &Task1::scanCallback, this);

//--------------Index 초기화-------------------------------|
  first_l = 0.2;
  first_r = -0.2;

  before_angle = 0;

  current_speed_ = 0;

  target_index_ = 0;

  r_rx.resize(1);
  r_ry.resize(1);
  r_ryaw.resize(1);
  r_rk.resize(1);

  l_rx.resize(1);
  l_ry.resize(1);
  l_ryaw.resize(1);
  l_rk.resize(1);
}

void Task1::vector_clear(){
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
  
  //----------------CONE CONE CONE CONE CONE CONE CONE CONE CONE CONE------------------------|
void Task1::decision(){
  vector<vector<double>> cloud_l, cloud_r;
  vector<double> x, y, rx, ry, ryaw, rk; 

  int first_left_index = 0, first_right_index = 0;
  double yy;
  bool left_flag = false, right_flag = false;

  for(int i = 0; i < center.size();i++){  // index 넣어주기
    center_index.push_back(i);
  }

  for (size_t i = 0; i < center_index.size(); i++)  // 좌,우 판단 후 index 넣어주기
  {
    PointType point;

    yy = center[center_index[i]].y;

    if(yy > 0 && !left_flag){           // y좌표로 좌우 판단  
      left_flag = true;
      first_left_index = i;             // index 넣어주기(최초 1번만 실행)
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
  left_index.push_back(first_left_index);         // 처음 점 넣기
  for(int i = 0; i < center_index.size(); i++){
    if(is_in_vector(left_index,i)){               // left_index에 i 가 있으면 ture, 없으면 false
      continue;
    }
    double temp_x = center[left_index.back()].x;
    double temp_y = center[left_index.back()].y;
    int compare = left_index.back();
    int index = 0;
    double min_dist = 9999;
    for(int j = compare; j < center_index.size(); j++){
      double now_dist = sqrt(pow(temp_x - center[center_index[j]].x,2) + pow(temp_y - center[center_index[j]].y,2));
      if(min_dist > now_dist && !is_in_vector(left_index,j)){   // 가장 가까운 거리 index 추출
        min_dist = now_dist;
        index = j;
      }
    }
    if(sqrt(pow(temp_x - center[center_index[index]].x,2) + pow(temp_y - center[center_index[index]].y,2)) > cone_dist_){ // 왼쪽 점과 가까운 점의 거리가 3.0 보다 크면(오른쪽 점)
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
    double temp_x = center[right_index.back()].x;
    double temp_y = center[right_index.back()].y;
    int compare = right_index.back();
    int index = 0;
    double min_dist = 9999;
    for(int j = compare; j < center_index.size(); j++){
      double now_dist = sqrt(pow(temp_x - center[center_index[j]].x,2) + pow(temp_y - center[center_index[j]].y,2));
      if(min_dist > now_dist && !is_in_vector(right_index,j)){
        min_dist = now_dist;
        index = j;
      }
    }
    if(sqrt(pow(temp_x - center[center_index[index]].x,2) + pow(temp_y - center[center_index[index]].y,2)) > cone_dist_){
      break;
    }
    else{
      right_index.push_back(index);
    }
  }


  
  vector<int> duplicates;
  if(center.size()!=0){
    for (int value : left_index) { // left index의 요소를 value에 할당(left index의 요소 수 만큼 반복)
      if (std::find(right_index.begin(), right_index.end(), value) != right_index.end()) {  // 중복 index 확인
        duplicates.push_back(value);  // duplicates : 좌우 중복으로 분류된 index
      }
    }
    sort(duplicates.begin(), duplicates.end(),duplicates_track_cmp);  //  작은 숫자부터 정렬
    float min_dist = 9999;
    int min_index;
    for(int i = 0; i < duplicates.size();i++){
      vector<int> min_duplicates;
      int value = duplicates[i];  // value : 좌우 중복으로 분류된 index 값
      if(fabs(center[first_left_index].y - center[duplicates[i]].y) - fabs(center[first_right_index].y - center[duplicates[i]].y) > 1.2){ // 중복된 index의 좌표가 오른쪽 보다 왼쪽 첫 점과 거리 차이가 많이 나면서 1.2 초과일 때(오른쪽 점일 때)
        left_index.erase(std::remove(left_index.begin(), left_index.end(), value), left_index.end());                                     // 왼쪽에서 지워줌
        cout << fabs(center[first_left_index].y - center[duplicates[i]].y) - fabs(center[first_right_index].y - center[duplicates[i]].y) << endl;
        cout << "index " << duplicates[i] << "right" << endl;
      }
      else if (fabs(center[first_left_index].y - center[duplicates[i]].y) - fabs(center[first_right_index].y - center[duplicates[i]].y) < -1.2){  // 중복된 index의 좌표가 왼쪽 보다 오른쪽 첫 점과 거리 차이가 많이 나면서 -1.2 미만일 때(왼쪽 점일 때)
        right_index.erase(std::remove(right_index.begin(), right_index.end(), value), right_index.end());                                         // 오른쪽에서 제거
        cout << fabs(center[first_left_index].y - center[duplicates[i]].y) - fabs(center[first_right_index].y - center[duplicates[i]].y) << endl;
        cout << "index " << duplicates[i] << "left" << endl;
      } 
      else if(fabs(fabs(center[first_left_index].y - center[duplicates[i]].y) - fabs(center[first_right_index].y - center[duplicates[i]].y)) <= 1.2){ // 중복인데 거리 차이가 1.2 이하일 때
        cout << "what!!!!!?????" << endl;
        cout << "index " << duplicates[i] << endl;
        for(int j=0; j < value; j++){
          if(min_dist > sqrt(pow(center[duplicates[i]].x - center[center_index[j]].x,2) + pow(center[duplicates[i]].y - center[center_index[j]].y,2))){ // 중복된 점 중 거리가 1.2 이하인 가장 가까운 점 찾기
            min_dist = sqrt(pow(center[duplicates[i]].x - center[center_index[j]].x,2) + pow(center[duplicates[i]].y - center[center_index[j]].y,2));
            min_index = j;    // 중복된 것 중에서 1.2 이하인 인덱스 넣기
          }
        }
        cout << "min dist: " << min_dist << endl; 
        cout << "min index: " << min_index << endl;
        if (std::find(right_index.begin(), right_index.end(), min_index) != right_index.end()) {  // min_index가 right_index에 있을 때
            min_duplicates.emplace_back(min_index);
        }

        if(min_duplicates.size() == 0) {
          right_index.erase(std::remove(right_index.begin(), right_index.end(), value), right_index.end()); // value값(전체 index 중 중복된 index) 찾아서 제거
          cout << "hahah this is left"<< endl;
        }
        else if(min_duplicates.size() != 0){
          left_index.erase(std::remove(left_index.begin(), left_index.end(), value), left_index.end()); // value값(전체 index 중 중복된 index) 찾아서 제거
          cout << "hahah this is right" << endl;
        }
        min_dist = 9999;
      }
    }
  }

  if(center.size() > 0){
    for(int i = 0; i < left_index.size();i++){          // 왼쪽 cone 좌표 넣기
      outline_cones.push_back(center[left_index[i]]);
    }
    for(int i = 0; i < right_index.size();i++){         // 오른쪽 cone 좌표 넣기
      inline_cones.push_back(center[right_index[i]]);
    }
  }


  sort(outline_cones.begin(),outline_cones.end(),track_cmp);  // x좌표 기준 정렬
  sort(inline_cones.begin(),inline_cones.end(),track_cmp);    // x좌표 기준 정렬
  cout << "center size       : " << center.size() << endl;
  cout << "inline cone size  : " << inline_cones.size() << endl;
  cout << "outline cone size : " << outline_cones.size() << endl;
}
//여기까지가 왼쪽과 오른쪽 점들 구분하는 과정
//out => left , in => right

//여기부터 길만들기
void Task1::make_line() {
  vector<double> input_x_l, input_y_l,input_x_r,input_y_r;
  input_x_l.clear();
  input_y_l.clear();
  input_x_r.clear();
  input_y_r.clear();
  double ds = 0.01;

  if(outline_cones.size() > 0){ // 처음 점 임의로 생성
      input_x_l = {-0.2};
      input_y_l = {first_l};  // first_l = 0.2
  }
  if(inline_cones.size() > 0){
      input_x_r = {-0.2};
      input_y_r = {first_r};  // first_r = -0.2
  }


  for (size_t i = 0; i < outline_cones.size(); i++) // 최종 왼쪽 좌표 넣어주기
  {
    double x,y;
    x = outline_cones[i].x;
    y = outline_cones[i].y;
    input_x_l.push_back(x);
    input_y_l.push_back(y);
  }

  for (size_t i = 0; i < inline_cones.size(); i++) // 최종 오른쪽 좌표 넣어주기
  {
    double x,y;
    x = inline_cones[i].x;
    y = inline_cones[i].y;
    input_x_r.push_back(x);
    input_y_r.push_back(y);
  }

  //-----------------------------예외처리 코드 왼쪽이 부족할때---------------------------
  if (outline_cones.size() <= 1){
    cout << "왼쪽 방어코드 실행" << endl;
    input_x_l.clear();
    input_y_l.clear();

    if(inline_cones.size() >=3){    //오른쪽이 세개이상 찍혔으면 그냥 밀어서 왼쪽에 사용
      for(int i = 0; i < input_x_r.size(); i++){  
          input_x_l.push_back(input_x_r[i]);
          input_y_l.push_back(input_y_r[i] + dist_offset_); //-------------dist_offset_ 파라미터---------------
      }  
    }
    else{      //오른쪽이 세개이하 찍혔으면 그냥 임의로 넣어줌
      input_x_l.resize(2);
      input_y_l.resize(2);
      input_x_l = {0.0,0.3};                  //--------------수정---------------------
      input_y_l = {0.2,0.2};                  //--------------수정---------------------
    }
    calc_spline_course(input_x_l, input_y_l, l_rx, l_ry, l_ryaw, l_rk, ds); // 출력(좌표) : l_rx,l_ry
  }   //ds 만큼 간격으로 x좌표들이 촘촘하게 경로따라 찍히는데 각각 l_rx,l_ry 여기에 저장
  else{         //l_ryaw는 경로를 따라 이동할때의 방향(각도)이 저장
    calc_spline_course(input_x_l, input_y_l, l_rx, l_ry, l_ryaw, l_rk, ds); 
  }             //l_rk는 경로에서 곡률값(회전정도)들이 저장, 클수록 경로가 많이 굽어있다                                                             
  
                                                              
  //-----------------------------예외처리 코드 오른쪽이 부족할때---------------------------
  if (inline_cones.size() <= 1){//예외처리 코드
    cout << "오른쪽 방어코드 실행" << endl;
    input_x_r.clear();
    input_y_r.clear();

    if(outline_cones.size() >=3){
      for(int i = 0; i < input_x_l.size(); i++){  // 왼쪽 점 밀어서 오른쪽에 사용
          
          input_x_r.push_back(input_x_l[i]);
          input_y_r.push_back(input_y_l[i] - dist_offset_);
      }
      
    }
    else{ // 임의로 넣어줌
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
  //여기까지가 차가 가야하는 길을 벡터들에 저장한 과정


  //여기서부터 길을 rviz 에 펍해준다
  nav_msgs::Path path_msg_l;
  nav_msgs::Path path_msg_r;

  path_msg_l.header.stamp = ros::Time::now();
  path_msg_l.header.frame_id = frame_id;
  for (size_t i = 0; i < l_rx.size(); i++)
  {
      geometry_msgs::PoseStamped pose;
      pose.pose.position.x = l_rx[i]; // spline 돌린 값 넣어줌
      pose.pose.position.y = l_ry[i];
      pose.pose.orientation.w = 1.0;  //경로상의 방향을 나타내는 값 현재 회전이 없는 기본값
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


  if (r_ry.size() >= 3){  // 첫 점 3개 묶어서 사용(평균값), 2번째 루프부터
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

bool Task1::is_in_vector(vector<int> v, int element){
    vector<int>::iterator it;                 // vector 반복자 it 선언
    it = find(v.begin(), v.end(), element);   // element를 찾을 때 까지 반복
    if (it != v.end()) {                      // 끝까지 찾았을 때 찾은 위치가 end가 아니면(element가 있으면)
        return true;
    } else {
        return false;
    }
}

void Task1::make_waypoint(){    //size = (int)(rx.size()*0.5);
  vector<double> input_x, input_y,rx,ry,ryaw,rk;
  double ds = 0.01;
  int angle = 0;
  float speed = 0.0f;
  int size_ = (l_rx.size() >= r_rx.size()) ? r_rx.size() : l_rx.size(); //왼쪽, 오른쪽 중 작은 사이즈를 선택
  int count = 0;
  double sum_x = 0, sum_y = 0;
  int size;
    
  //중앙경로 생성
  for (int i = 0; i < size_; i++){
    if(count%3 == 0){
      sum_x += (l_rx[i] + r_rx[i])/2; // 좌우 평균값
      sum_y += (l_ry[i] + r_ry[i])/2;
      // 3 << "sum_x : : " << sum_x << endl;
      input_x.push_back(sum_x/3); // 3개씩 묶어서 평균값
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
    input_x.push_back((l_rx[l_rx.size()-1] + r_rx[r_rx.size()-1])/2); // 마지막 인덱스 값 평균
    input_y.push_back((l_ry[l_ry.size()-1] + r_ry[r_ry.size()-1])/2);
  }

  input_x.erase(input_x.begin());
  input_y.erase(input_y.begin());

  if (input_x.size() < 2){//예외처리 코드
    input_x = {0.0,0.3};  // 임의로 넣어줌
    input_y = {0.0,0.0};
    calc_spline_course(input_x,input_y,rx,ry,ryaw,rk,ds); 
  }
  else{
    calc_spline_course(input_x,input_y,rx,ry,ryaw,rk,ds);
  }

  visualize_path(rx, ry, frame_id);

  size = (int)(rx.size()*0.5);

// --------------------------주행-------------------------------------------------------------------------//
  //cout << "waypoint size" << rx.size() << endl
// -----------------------------------

  geometry_msgs::PoseStamped target_p;

  if (rx.size() > 10  ) // 속력 설정
  {
  //------------define angle---------
    //angle = pure_pursuit(rx, ry, target_p,speed);
    // angle = stanley(rx,ry,ryaw,rk);
    angle = pure_pursuit(rx, ry, target_p,speed,rk);

    before_angle = GetAverage(avg_vec_);

    speed_total_ = 0.1;
  }
  else{
    cout << "방어 steer" << endl;
    angle = before_angle;

    speed_total_ = 0.1;
  }

  if(sqrt(pow(target_p.pose.position.x,2)+pow(target_p.pose.position.y,2))< 0.05){
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

  cout << "ANGLEEEEEEEEEEEEEEEEEE: " << angle << endl;
  steer_total_ = angle;

  cmd_vel_msg.drive.speed = speed_total_;
  cmd_vel_msg.drive.steering_angle = steer_total_;

  cmd_vel.publish(cmd_vel_msg);

  cout << "speed_total_  = " << speed_total_ << endl; 
  cout << "steer_total_  = " << steer_total_ << endl;
  cout << "rx.size= " << rx.size() <<endl;
  cout << "\n" <<endl;

  avg_vec_.push_back(angle);
  
  if(avg_vec_.size() > 10){ avg_vec_.erase(avg_vec_.begin());}

}

int Task1::GetAverage(vector<double> const& vec){
    if (vec.empty()){
        return 0;
    }
    return accumulate(vec.begin(),vec.end(),0.0) / vec.size();
}
int Task1::pure_pursuit(vector<double>& rx, vector<double>& ry, geometry_msgs::PoseStamped& target_p,float& speed,vector<double>& rk)
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
  
  for (int i = 0; i < rx.size(); i++){
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
  if (final_ld > 3.0f)
    final_ld = 3.0f;
  if (final_ld < 2.2f)
    final_ld = 2.2f;

  float cur_steer = atan2f(2.0f * 0.18f * sinf(alpha) / (final_ld), 1.0f) * -180.0f / M_PI ; // 1.04f: 휠베이스
  cur_steer *= 0.5;

  cout << "===== Pure-pursuit ====="<< endl;
  cout << "target_index_" << target_index_<<endl;
  cout << "lookahead distance   : " << lookahead_dist <<endl;
  cout << "final ld             : " << final_ld <<endl;
  cout << "pure pursuit steer   : " << cur_steer << endl;
  cout << "speed: "<< cmd_vel_msg.drive.speed<<endl;
  cout << "steer: "<< cmd_vel_msg.drive.steering_angle << endl;
  cout << "dist: " << sqrt(pow(target_p.pose.position.x,2)+pow(target_p.pose.position.y,2)) <<endl;

  return cur_steer;
}

void Task1::run(){
    vector_clear();
    set_roi(msg_, roi);
    voxel(msg_, voxel_size);
    clustering(msg_, center, param);
    cout << "center_size : " << center.size() << endl;
    decision();
    // visualize_rviz(center, frame_id , 0);
    rviz_clear(frame_id);
    visualize_rviz(outline_cones, frame_id , 1);
    visualize_rviz(inline_cones, frame_id , 2);
    make_line();
    make_waypoint(); 
  
}