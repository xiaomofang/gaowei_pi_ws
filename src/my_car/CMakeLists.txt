cmake_minimum_required(VERSION 3.0.2)
project(my_car)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  message_generation
  std_msgs
)

find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)

add_message_files(
  FILES
  Speed.msg
  # Message2.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include 
#  LIBRARIES my_car
  CATKIN_DEPENDS roscpp serial  std_msgs message_runtime
#  DEPENDS system_lib
)

include_directories(
  #include
  ${catkin_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  
)

include_directories(include ${OpenCV_INCLUDE_DIRS})


add_executable(path_plan src/path.cpp 
                    src/Astar_searcher.cpp
                    src/Navigation.cpp)
target_link_libraries(path_plan ${catkin_LIBRARIES})

add_executable(car_ctl src/Car_Ctl.cpp
                       src/Serial_Send.cpp)
target_link_libraries(car_ctl ${catkin_LIBRARIES} ${OpenCV_LIBS})
