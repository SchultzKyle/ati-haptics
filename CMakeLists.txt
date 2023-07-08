cmake_minimum_required(VERSION 2.8.3)


project(ati_haptics)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg geometry_msgs turtlesim message_filters)

catkin_package()

find_package(roslaunch)

find_package(Boost REQUIRED COMPONENTS thread)
include_directories(include 
	${catkin_INCLUDE_DIRS}
)
	
add_executable(ati_haptics scripts/ati_haptics.cpp)
target_link_libraries(ati_haptics ${catkin_LIBRARIES}
	dhd
	usb-1.0 -lrt
	pthread)
	
foreach(dir config launch meshes urdf rviz)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
