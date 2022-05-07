cmake_minimum_required(VERSION 2.8)

project(autoCalibration)

if (POLICY CMP0072)
  set(OpenGL_GL_PREFERENCE LEGACY)
endif()

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_MODULE_PATH "/path")
set(CMAKE_CXX_FLAGS "-std=c++11 -O3")
set(CMAKE_CXX_STANDARD 14)

find_package( OpenCV 3.1 REQUIRED )
find_package( Sophus REQUIRED )
set(Sophus_LIBRARIES libSophus.so)

include_directories( 
    ${OpenCV_INCLUDE_DIRS}
    ${G2O_INCLUDE_DIRS}
    "/usr/include/eigen3/"
	${Sophus_INCLUDE_DIRS}
)

add_library(calibration calibration.cpp)
add_library(calibrationShared SHARED calibration.cpp)
target_link_libraries(calibration 
	${Sophus_LIBRARIES})
add_library(line_extractor line_extractor.cpp)
add_library(line_extractorShared SHARED line_extractor.cpp)
target_link_libraries(line_extractor 
	${OpenCV_LIBS}
	${Sophus_LIBRARIES})

	


	
add_executable(autoCalibration main.cpp)
target_link_libraries(autoCalibration
	${OpenCV_LIBS}
	calibration
	line_extractor
	${Sophus_LIBRARIES})









