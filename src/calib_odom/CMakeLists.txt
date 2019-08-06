cmake_minimum_required(VERSION 2.8.3)
project(calib_odom)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_filters
)

find_package(Boost REQUIRED COMPONENTS system thread)

find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp
)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(calib_odom_node
  src/calib_odom_node.cpp
  src/calib_odom.cpp
)
target_link_libraries(calib_odom_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(marker_pub_node
  src/marker_pub_node.cpp
)
target_link_libraries(marker_pub_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(odom_result_node
  src/odom_result_node.cpp
)
target_link_libraries(odom_result_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS calib_odom_node marker_pub_node odom_result_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)