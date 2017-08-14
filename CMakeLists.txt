cmake_minimum_required(VERSION 2.8)
project( OpenCV_Wrappers )
find_package( OpenCV REQUIRED )
set(VTK_DIR "D:/Downloads/extracted/VTK-6.3.0/build")
find_package(VTK REQUIRED)

#include opencv include directory
include_directories( ${OpenCV_INCLUDE_DIRS} )
#include vtk include directory
include_directories(${VTK_INCLUDE_DIRS})
include_directories(include)
include_directories(${CMAKE_SOURCE_DIR}/cv_wrappers/include/)
link_directories(${CMAKE_SOURCE_DIR}/lib)




#add custum CV wrappers
add_subdirectory(cv_wrappers/)

#add tests

#include additional OPENCV directories
add_executable(OpenCV_Wrappers ${CMAKE_SOURCE_DIR}/tests/opencv_example.cpp )

#add LK test
add_executable(LK_TEST ${CMAKE_SOURCE_DIR}/tests/test_LK.cpp)

#add viz test for visualization
add_executable(VIZ_TEST ${CMAKE_SOURCE_DIR}/tests/test_VIZ.cpp)

#add test for hough circle detector
add_executable(HOUGH_CIRCLE_TEST ${CMAKE_SOURCE_DIR}/tests/test_hough_circle.cpp)

#add test for stere
add_executable(STEREO_TEST ${CMAKE_SOURCE_DIR}/tests/test_stereo.cpp)

#add Opencv3.2, custom cv wrapper and ovrvision libraries
target_link_libraries(OpenCV_Wrappers ${OpenCV_LIBS} CV_CUSTOM_WRAPPER  ovrvision64d )
target_link_libraries(LK_TEST ${OpenCV_LIBS} CV_CUSTOM_WRAPPER  ovrvision64d )
target_link_libraries(VIZ_TEST ${OpenCV_LIBS}  CV_CUSTOM_WRAPPER)
target_link_libraries(HOUGH_CIRCLE_TEST ${OpenCV_LIBS}  CV_CUSTOM_WRAPPER)
target_link_libraries(STEREO_TEST ${OpenCV_LIBS}  CV_CUSTOM_WRAPPER)
file(GLOB dlls_vtk ${VTK_DIR}/bin/debug/*.dll)
file(GLOB dlls_cv ${OpenCV_DIR}/bin/debug/*.dll)
file(GLOB dlls_extra ${CMAKE_SOURCE_DIR}/bin/*.dll)
file(GLOB camera_info ${CMAKE_SOURCE_DIR}/camera_info/*.xml)
add_custom_target(add_dlls

	COMMAND ${CMAKE_COMMAND} -E copy ${dlls_cv}          ${CMAKE_BINARY_DIR}/Debug/
	COMMAND ${CMAKE_COMMAND} -E copy ${dlls_vtk}          ${CMAKE_BINARY_DIR}/Debug/
	COMMAND ${CMAKE_COMMAND} -E copy ${dlls_extra}         ${CMAKE_BINARY_DIR}/Debug/
	COMMAND ${CMAKE_COMMAND} -E copy ${camera_info}         ${CMAKE_BINARY_DIR}/Debug/

)

add_dependencies(OpenCV_Wrappers add_dlls)
add_dependencies(LK_TEST add_dlls)
add_dependencies(VIZ_TEST add_dlls)
add_dependencies(HOUGH_CIRCLE_TEST add_dlls)
add_dependencies(STEREO_TEST add_dlls)



