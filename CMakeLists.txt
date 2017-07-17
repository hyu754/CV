cmake_minimum_required(VERSION 2.8)
project( LibCV )
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories(include)
link_directories(${CMAKE_SOURCE_DIR}/lib)
#include additional OPENCV directories

add_executable( LibCV opencv_example.cpp )




target_link_libraries( LibCV ${OpenCV_LIBS} ovrvision64d )



add_custom_target(add_dlls

	COMMAND ${CMAKE_COMMAND} -E copy ${OpenCV_DIR}/bin/debug/opencv_core330d.dll             ${CMAKE_BINARY_DIR}/Debug/
	COMMAND ${CMAKE_COMMAND} -E copy ${OpenCV_DIR}/bin/debug/opencv_highgui330d.dll          ${CMAKE_BINARY_DIR}/Debug/
	COMMAND ${CMAKE_COMMAND} -E copy ${OpenCV_DIR}/bin/debug/opencv_videoio330d.dll          ${CMAKE_BINARY_DIR}/Debug/
	COMMAND ${CMAKE_COMMAND} -E copy ${OpenCV_DIR}/bin/debug/opencv_imgcodecs330d.dll          ${CMAKE_BINARY_DIR}/Debug/
	COMMAND ${CMAKE_COMMAND} -E copy ${OpenCV_DIR}/bin/debug/opencv_imgproc330d.dll          ${CMAKE_BINARY_DIR}/Debug/
	COMMAND ${CMAKE_COMMAND} -E copy ${OpenCV_DIR}/bin/debug/opencv_xfeatures2d330d.dll          ${CMAKE_BINARY_DIR}/Debug/
	COMMAND ${CMAKE_COMMAND} -E copy ${OpenCV_DIR}/bin/debug/opencv_cudafeatures2d330d.dll          ${CMAKE_BINARY_DIR}/Debug/
	COMMAND ${CMAKE_COMMAND} -E copy ${OpenCV_DIR}/bin/debug/opencv_features2d330d.dll          ${CMAKE_BINARY_DIR}/Debug/
	COMMAND ${CMAKE_COMMAND} -E copy ${OpenCV_DIR}/bin/debug/opencv_cudafilters330d.dll          ${CMAKE_BINARY_DIR}/Debug/
	COMMAND ${CMAKE_COMMAND} -E copy ${OpenCV_DIR}/bin/debug/opencv_cudawarping330d.dll          ${CMAKE_BINARY_DIR}/Debug/
	COMMAND ${CMAKE_COMMAND} -E copy ${OpenCV_DIR}/bin/debug/opencv_cudaarithm330d.dll          ${CMAKE_BINARY_DIR}/Debug/
	COMMAND ${CMAKE_COMMAND} -E copy ${OpenCV_DIR}/bin/debug/opencv_flann330d.dll          ${CMAKE_BINARY_DIR}/Debug/
	
	COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_SOURCE_DIR}/bin/ovrvisiond.dll          ${CMAKE_BINARY_DIR}/Debug/

)

add_dependencies(LibCV add_dlls)
