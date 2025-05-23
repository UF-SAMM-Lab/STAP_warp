cmake_minimum_required(VERSION 2.8.3)
project(stap_warp)
add_compile_options(-std=c++17 -funroll-loops -Wall -Ofast)
# set(CMAKE_BUILD_TYPE Release)
add_definitions(-w)
#add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  graph_core
  moveit_core
  moveit_ros_planning
  roscpp
  geometry_msgs
  irrt_star_avoid
  # edo_description_simple
  visualization_msgs
  subscription_notifier
  human_probablistic_occupancy
  moveit_msgs
  actionlib
  actionlib_msgs
  velocity_scaling_iso15066
  message_generation
  jsk_rviz_plugins
  manipulation_msgs
)

find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(Torch REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -Wextra")

add_library(inverse_kinematics
  src/inverse_kinematics/ik_seed_search.cpp
)

add_service_files(
  FILES
  human_prediction.srv
  human_motion_done.srv
  human_motion_reset.srv
)

add_message_files(
  FILES
  joint_seq_elem.msg
  joint_seq.msg
)

message(STATUS "generating messages")
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    graph_core
    moveit_core
    moveit_ros_planning
    roscpp
    geometry_msgs
    irrt_star_avoid
    # edo_description_simple
    visualization_msgs
    subscription_notifier
    human_probablistic_occupancy
    velocity_scaling_iso15066
  DEPENDS Boost
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/sequence.cpp
  src/${PROJECT_NAME}/stap_warp.cpp
  src/${PROJECT_NAME}/utilities.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  torch
)

add_executable(test_sim src/test_sim.cpp)
target_link_libraries(test_sim
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
  torch
)
add_dependencies(test_sim ${catkin_EXPORTED_TARGETS})
install(TARGETS test_sim DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(test_live src/test_live.cpp)
target_link_libraries(test_live
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
  torch
)
add_dependencies(test_live ${catkin_EXPORTED_TARGETS})
install(TARGETS test_live DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(blend_test src/blend_test.cpp)
target_link_libraries(blend_test
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
  torch
)
add_dependencies(blend_test ${catkin_EXPORTED_TARGETS})
install(TARGETS blend_test DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(sequence_test src/sequence_test.cpp)
target_link_libraries(sequence_test
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
  torch
)
add_dependencies(sequence_test ${catkin_EXPORTED_TARGETS})
install(TARGETS sequence_test DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(sw_get_angles src/sharework_get_angles.cpp)
target_link_libraries(sw_get_angles
  ${catkin_LIBRARIES}
  inverse_kinematics
)
add_dependencies(sw_get_angles ${catkin_EXPORTED_TARGETS})
install(TARGETS sw_get_angles DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(sw_test_pos src/sharework_test_position.cpp)
target_link_libraries(sw_test_pos
  ${catkin_LIBRARIES}
  inverse_kinematics
)
add_dependencies(sw_test_pos ${catkin_EXPORTED_TARGETS})
install(TARGETS sw_test_pos DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(sw_go_pos src/sharework_go_to_position.cpp)
target_link_libraries(sw_go_pos
  ${catkin_LIBRARIES}
)
add_dependencies(sw_go_pos ${catkin_EXPORTED_TARGETS})
install(TARGETS sw_go_pos DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(sw_test_grip src/sharework_test_gripper.cpp)
target_link_libraries(sw_test_grip
  ${catkin_LIBRARIES}
)
add_dependencies(sw_test_grip ${catkin_EXPORTED_TARGETS})
install(TARGETS sw_test_grip DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


add_executable(sw_grip_int src/sharework_gripper_interpretter.cpp)
target_link_libraries(sw_grip_int
  ${catkin_LIBRARIES}
)
add_dependencies(sw_grip_int ${catkin_EXPORTED_TARGETS})
install(TARGETS sw_grip_int DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(load_scene src/load_sequence_scene.cpp)
target_link_libraries(load_scene
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)
add_dependencies(load_scene ${catkin_EXPORTED_TARGETS})
install(TARGETS load_scene DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_dirrt_star.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
