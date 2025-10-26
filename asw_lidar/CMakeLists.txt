cmake_minimum_required(VERSION 3.0.2)
project(asw_lidar)

# 패키지 종속성
find_package(catkin REQUIRED
  geometry_msgs
  roscpp
  std_msgs
  sensor_msgs
  visualization_msgs
  message_generation
  pcl_ros
  cv_bridge
  laser_geometry
)

find_package(Boost REQUIRED COMPONENTS system)

# 메시지 파일 추가
add_message_files(
  FILES
  obs.msg  # obs.msg 파일을 메시지 파일 목록에 추가
)

# 메시지 생성
generate_messages(
  DEPENDENCIES
  std_msgs  # obs.msg 파일에서 종속되는 메시지 패키지
)

# catkin 패키지 설정
catkin_package(
  INCLUDE_DIRS include 
  LIBRARIES Mando_lidar
  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs visualization_msgs pcl_ros std_msgs laser_geometry message_runtime  # message_runtime 추가
)

# include 디렉토리 설정
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# 실행 파일 추가
add_executable(${PROJECT_NAME}_node src/main.cpp) 

# 링크 라이브러리 설정
target_link_libraries(${PROJECT_NAME}_node 
  ${catkin_LIBRARIES}
)

# 메시지 및 빌드 의존성 추가
add_dependencies(${PROJECT_NAME}_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

