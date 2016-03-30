cmake_minimum_required(VERSION 2.8.3)
project(motoman_grippers)

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  )

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/include/)


SET(SOURCES ${PROJECT_SOURCE_DIR}/src/gpio.cpp
            ${PROJECT_SOURCE_DIR}/src/schunk_grippers.cpp
            ${PROJECT_SOURCE_DIR}/src/motoman_grippers.cpp )

SET(HEADERS ${PROJECT_SOURCE_DIR}/include/schunk_grippers.h
            ${PROJECT_SOURCE_DIR}/include/gpio.h )

catkin_package(
  DEPENDS
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp 
  LIBRARIES
   
)

add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${ROS_LIBRARIES} ${catkin_LIBRARIES})
