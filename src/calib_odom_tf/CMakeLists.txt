cmake_minimum_required(VERSION 2.8.3)
project(calib_odom_tf)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_filters
  roscpp
  tf2_msgs
  nav_msgs
)

# find_package(Boost REQUIRED COMPONENTS system thread)

find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS 
  roscpp 
  message_filters
  tf2_msgs
  nav_msgs
)

include_directories(
  include
#  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(calib_odom_tf_node 
  src/calib_odom_node.cpp
  src/calib_odom_tf.cpp
)
target_link_libraries(calib_odom_tf_node
  ${catkin_LIBRARIES}
#  ${Boost_LIBRARIES}
)

add_executable(pub_tf_stamp_node
  src/pub_tf_stamp_node.cpp
)
target_link_libraries(pub_tf_stamp_node
  ${catkin_LIBRARIES}
  # ${Boost_LIBRARIES}
)

add_executable(odom_result_node
  src/odom_result_node.cpp
)
target_link_libraries(odom_result_node
  ${catkin_LIBRARIES}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS  pub_tf_stamp_node calib_odom_tf_node odom_result_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)