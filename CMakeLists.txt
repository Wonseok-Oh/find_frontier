cmake_minimum_required(VERSION 2.8.3)
project(find_frontier)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

set(OpenCV_DIR [OpenCV.cmake PATH])
find_package(OpenCV REQUIRED
   COMPONENTS
   opencv_photo
)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  std_msgs
  move_base_msgs
  actionlib
  geometry_msgs
  cv_bridge
  costmap_2d
  navfn
  nav_core
  pluginlib
  tf2
  tf2_ros
  tf2_geometry_msgs
  global_planner
  message_generation
)

add_service_files(
  FILES
  InitPos.srv
  InitPosList.srv
)

add_message_files(
  FILES
  ActionArray.msg
)

generate_messages (
  DEPENDENCIES
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    cv_bridge geometry_msgs roscpp rospy tf2 tf2_ros std_msgs message_runtime
  DEPENDS
    OpenCV
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIR}
)
add_library(find_frontier src/find_frontier.cpp)
target_link_libraries(find_frontier
   ${Boost_LIBRARIES}
   ${catkin_LIBRARIES}
)

add_library(find_frontier_multi src/find_frontier_multi.cpp)
target_link_libraries(find_frontier_multi
   ${Boost_LIBRARIES}
   ${catkin_LIBRARIES}
)

add_dependencies(find_frontier ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} find_frontier_gencpp)
add_dependencies(find_frontier_multi ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} find_frontier_multi_gencpp)

add_executable(find_frontier_node src/find_frontier_node.cpp)
add_dependencies(find_frontier_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(find_frontier_node find_frontier)

add_executable(find_frontier_multi_node src/find_frontier_multi_node.cpp)
add_dependencies(find_frontier_multi_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(find_frontier_multi_node find_frontier_multi)

add_executable(potential_printer src/potential_printer.cpp)
add_dependencies(potential_printer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(potential_printer ${catkin_LIBRARIES})


add_executable(objmap_to_image_converter src/objmap_to_image_converter.cpp)
target_link_libraries(objmap_to_image_converter 
   ${catkin_LIBRARIES} 
   ${OpenCV_LIBRARIES}
)
#add_library(global_planner src/global_planner.cpp)
#target_link_libraries(global_planner
#   ${Boost_LIBRARIES}
#   ${catkin_LIBRARIES}
#)
#add_dependencies(global_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#add_executable(global_planner_node src/global_planner_node.cpp)
#add_dependencies(global_planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(global_planner_node global_planner)

#install(
#    TARGETS
#        global_planner_node
#    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

#install(
#    TARGETS
#        global_planner
#    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
