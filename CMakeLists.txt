cmake_minimum_required(VERSION 2.8.3)
project(student_proect)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package( PkgConfig REQUIRED)
pkg_search_module( TESSERACT REQUIRED tesseract )
pkg_search_module( LEPTONICA REQUIRED lept )
find_package(OpenCV REQUIRED )
find_package(project_interface REQUIRED )

## Specify additional locations of header files
include_directories(
 include 
 ${OpenCV_INCLUDE_DIRS}
 ${TESSERACT_INCLUDE_DIRS}
 ${LEPTONICA_INCLUDE_DIRS} 
 ${project_interface_INCLUDE_DIRS}
)

link_directories( ${TESSERACT_LIBRARY_DIRS} )
link_directories( ${LEPTONICA_LIBRARY_DIRS} )

add_executable(calibrate
   src/camera_calibration.cpp
)

## CREATE EXECUTABLE LIBRARY

add_library(student SHARED
  src/my_utils.cpp
  src/transform.cpp
  src/map.cpp
  src/student_interface.cpp
)

target_link_libraries(student
  ${OpenCV_LIBRARIES}  
)
target_link_libraries(student
  ${TESSERACT_LIBRARIES}
)
target_link_libraries(student
  ${LEPTONICA_LIBRARIES} 
)

target_link_libraries(calibrate
  ${OpenCV_LIBRARIES}  
)

