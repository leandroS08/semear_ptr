cmake_minimum_required(VERSION 2.8.3)
project(semear_ptr)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  cv_bridge
  image_transport
  sensor_msgs
  std_msgs
  actionlib_msgs
  actionlib
)

find_package(OpenCV REQUIRED)

add_service_files(
  FILES
  Valvula.srv
  Painel.srv
  Cone.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)
#add_compile_options(-std=c++11)

catkin_package(DEPENDS OpenCV)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(painel_vision src/painel.cpp)
target_link_libraries(painel_vision
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
)

add_executable(cone_vision src/cone.cpp)
target_link_libraries(cone_vision
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
)

add_executable(valvula_vision src/valvula.cpp)
target_link_libraries(valvula_vision
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
)
