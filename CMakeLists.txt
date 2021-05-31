cmake_minimum_required(VERSION 3.0.2)
project(path_planning)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  roscpp
  rospy
  message_generation
  dynamic_reconfigure
  std_msgs
)

add_service_files(
   FILES
   input_params.srv
)

generate_messages(DEPENDENCIES std_msgs)

generate_dynamic_reconfigure_options(
   cfg/dyn_params.cfg
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS} ${OMPL_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_node src/path_planner.cpp)
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_gencfg)

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${OMPL_LIBRARIES})