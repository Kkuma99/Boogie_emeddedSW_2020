cmake_minimum_required(VERSION 2.8.0)
project(turtlebot3_manipulation_gui)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  geometry_msgs
  roscpp
  std_msgs
  sensor_msgs
  moveit_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  message_generation           # ros communication
)

find_package(Eigen3 REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(OpenCV REQUIRED)#####CV

set(${PROJECT_NAME}_SRCS
  src/main.cpp
  src/main_window.cpp
  src/qnode.cpp
)

set(${PROJECT_NAME}_HDRS
  include/${PROJECT_NAME}/main_window.hpp
  include/${PROJECT_NAME}/qnode.hpp
)

set(${PROJECT_NAME}_UIS
  ui/main_window.ui
)

set(${PROJECT_NAME}_INCLUDE_DIRECTORIES
  "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}/${PROJECT_NAME}"
)

if(NOT EXISTS ${${PROJECT_NAME}_INCLUDE_DIRECTORIES})
  file(MAKE_DIRECTORY ${${PROJECT_NAME}_INCLUDE_DIRECTORIES})
endif()

add_message_files(FILES msgTutorial.msg)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES turtlebot3_manipulation_gui
  CATKIN_DEPENDS
    geometry_msgs
    roscpp 
    std_msgs 
    sensor_msgs 
    moveit_msgs
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
  DEPENDS EIGEN3
  DEPENDS system_lib
)



qt5_wrap_cpp(${PROJECT_NAME}_MOCS     ${${PROJECT_NAME}_HDRS})
qt5_add_resources(${PROJECT_NAME}_RCC ${${PROJECT_NAME}_RESOURCES})

# ensure generated header files are being created in the devel space
set(_cmake_current_binary_dir "${CMAKE_CURRENT_BINARY_DIR}")
set(CMAKE_CURRENT_BINARY_DIR ${turtlebot3_manipulation_gui_INCLUDE_DIRECTORIES})

qt5_wrap_ui(${PROJECT_NAME}_UIS_H ${${PROJECT_NAME}_UIS})

set(CMAKE_CURRENT_BINARY_DIR "${_cmake_current_binary_dir}")


################################################################################
# Build
################################################################################
include_directories(
  include
  ${turtlebot3_manipulation_gui_INCLUDE_DIRECTORIES}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}#####CV
)

#ros_tutorial_msg_subscriber
add_executable(ros_tutorial_msg_subscriber src/ros_tutorial_msg_subscriber.cpp)
target_link_libraries(ros_tutorial_msg_subscriber ${catkin_LIBRARIES})
add_dependencies(ros_tutorial_msg_subscriber oroca_ros_tutorials_generate_messages_cpp)

add_executable(turtlebot3_manipulation_gui ${turtlebot3_manipulation_gui_SRCS} ${turtlebot3_manipulation_gui_MOCS} ${turtlebot3_manipulation_gui_UIS_H} ${turtlebot3_manipulation_gui_RCC})
add_dependencies(turtlebot3_manipulation_gui ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(turtlebot3_manipulation_gui Qt5::Widgets ${catkin_LIBRARIES} ${Eigen3_LIBRARIES})
target_link_libraries(turtlebot3_manipulation_gui ${catkin_LIBRARIES} ${OpenCV_LIBS})#####CV

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch ui
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

