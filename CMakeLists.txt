cmake_minimum_required(VERSION 2.8)
project( LibCV )
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

#include additional OPENCV directories
add_executable( LibCV opencv_example.cpp )

#add LK test
add_executable(LK_TEST test_LK.cpp)

#add viz test for visualization
add_executable(VIZ_TEST test_VIZ.cpp)



#add Opencv3.2, custom cv wrapper and ovrvision libraries
target_link_libraries( LibCV ${OpenCV_LIBS} CV_CUSTOM_WRAPPER  ovrvision64d )
target_link_libraries(LK_TEST ${OpenCV_LIBS} CV_CUSTOM_WRAPPER  ovrvision64d )
target_link_libraries(VIZ_TEST ${OpenCV_LIBS}  CV_CUSTOM_WRAPPER)
file(GLOB dlls_vtk ${VTK_DIR}/bin/debug/*.dll)


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
	COMMAND ${CMAKE_COMMAND} -E copy ${OpenCV_DIR}/bin/debug/opencv_video330d.dll          ${CMAKE_BINARY_DIR}/Debug/
	COMMAND ${CMAKE_COMMAND} -E copy ${OpenCV_DIR}/bin/debug/opencv_viz330d.dll          ${CMAKE_BINARY_DIR}/Debug/
	COMMAND ${CMAKE_COMMAND} -E copy ${OpenCV_DIR}/bin/debug/opencv_calib3d330d.dll          ${CMAKE_BINARY_DIR}/Debug/
	COMMAND ${CMAKE_COMMAND} -E copy ${OpenCV_DIR}/bin/debug/opencv_aruco330d.dll          ${CMAKE_BINARY_DIR}/Debug/
	#for cv::viz
	#COMMAND ${CMAKE_COMMAND} -E copy ${VTK_DIR}/bin/debug/vtkInteractionStyle-6.3.dll          ${CMAKE_BINARY_DIR}/Debug/
	#COMMAND ${CMAKE_COMMAND} -E copy ${VTK_DIR}/bin/debug/vtkRenderingVolumeOpenGL2-6.3.dll          ${CMAKE_BINARY_DIR}/Debug/
	#COMMAND ${CMAKE_COMMAND} -E copy ${VTK_DIR}/bin/debug/vtkRenderingLOD-6.3.dll          ${CMAKE_BINARY_DIR}/Debug/
	#COMMAND ${CMAKE_COMMAND} -E copy ${VTK_DIR}/bin/debug/vtkIOPLY-6.3.dll          ${CMAKE_BINARY_DIR}/Debug/
	#COMMAND ${CMAKE_COMMAND} -E copy ${VTK_DIR}/bin/debug/vtkIOGeometry-6.3.dll          ${CMAKE_BINARY_DIR}/Debug/
	#COMMAND ${CMAKE_COMMAND} -E copy ${VTK_DIR}/bin/debug/vtkFiltersTexture-6.3.dll          ${CMAKE_BINARY_DIR}/Debug/
	#COMMAND ${CMAKE_COMMAND} -E copy ${VTK_DIR}/bin/debug/vtkIOExport-6.3.dll          ${CMAKE_BINARY_DIR}/Debug/
	#COMMAND ${CMAKE_COMMAND} -E copy ${VTK_DIR}/bin/debug/vtkRenderingContextOpenGL-6.3.dll          ${CMAKE_BINARY_DIR}/Debug/
	#COMMAND ${CMAKE_COMMAND} -E copy ${VTK_DIR}/bin/debug/vtkRenderingOpenGL-6.3.dll          ${CMAKE_BINARY_DIR}/Debug/
	COMMAND ${CMAKE_COMMAND} -E copy ${dlls_vtk}          ${CMAKE_BINARY_DIR}/Debug/
	


	
	COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_SOURCE_DIR}/bin/ovrvisiond.dll          ${CMAKE_BINARY_DIR}/Debug/

)

add_dependencies(LibCV add_dlls)
add_dependencies(LK_TEST add_dlls)
add_dependencies(VIZ_TEST add_dlls)
