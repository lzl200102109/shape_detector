cmake_minimum_required(VERSION 2.8.3)
project(shape_detector)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  image_view
)

find_package(OpenCV REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################


###################################
## catkin specific configuration ##
###################################

catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(${catkin_INCLUDE_DIRS})

## Declare a cpp executable and specify libraries to link a library or executable target against
add_executable(shape_detector src/shape_detector_video.cpp
               src/feature.h
			   src/feature.cpp
			   src/geometry.h
			   src/geometry.cpp)
target_link_libraries(shape_detector
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
)
add_executable(shape_detector_image
			   src/shape_detector_image.cpp
               src/feature.h
			   src/feature.cpp
			   src/geometry.h
			   src/geometry.cpp)
target_link_libraries(shape_detector_image
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
)
add_executable(take_image src/take_image.cpp)
target_link_libraries(take_image
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

#############
## Testing ##
#############

