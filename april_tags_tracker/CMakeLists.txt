# Declare the version of the CMake API for forward-compatibility
cmake_minimum_required(VERSION 2.8)

# Declare the name of the CMake Project
project(april_tags_tracker)

# Find Catkin
find_package(catkin REQUIRED COMPONENTS
             roscpp
             sensor_msgs
             cv_bridge
             image_transport
             std_msgs
             message_generation
            )

#find_package(OpenCV 2.4 REQUIRED COMPONENTS core highgui imgproc calib3d
#                                            ml features2d objdetect )
find_package(cmake_modules)
find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)

# Find package apriltags
find_package(PkgConfig)
pkg_check_modules(APRILTAGS apriltags)
if( NOT APRILTAGS_FOUND )
  message( "Apriltags not found\n" )
  return()
endif()
find_path(APRILTAGS_INCLUDE_DIR AprilTags/TagDetector.h
    HINTS ${APRILTAGS_INCLUDEDIR} ${APRILTAGS_INCLUDE_DIRS})

find_library(APRILTAGS_LIBRARY apriltags
    HINTS ${APRILTAGS_LIBDIR} ${APRILTAGS_LIBRARY_DIRS})

add_message_files (
  FILES
  april_tag_pos.msg
)

generate_messages (
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

# Declare this project as a catkin package
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hello_world
   CATKIN_DEPENDS message_runtime
)

# Add the headers from roscpp
include_directories(include ${catkin_INCLUDE_DIRS}
                            ${OpenCV_INCLUDE_DIRS}
                            ${Eigen_INCLUDE_DIRS}
                            ${APRILTAGS_INCLUDE_DIRS}
                            )

add_executable(april_tags_tracker src/april_tags_tracker.cpp
                                  src/april_tags_tracker_demo.cpp
              )   
target_link_libraries(april_tags_tracker ${catkin_LIBRARIES} 
                                         ${OpenCV_LIBRARIES}
                                         ${APRILTAGS_LIBRARY}
                     )

install(TARGETS april_tags_tracker
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

