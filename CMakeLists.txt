cmake_minimum_required(VERSION 3.18)
project(coro_eyes_ros)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# coro_eyes_sdk
find_package(coro_eyes_sdk REQUIRED)
if(coro_eyes_sdk_FOUND)
	message("Found CoRo_Eyes_SDK")
else()
	message(FATAL_ERROR "Can't find CoRo_Eyes_SDK")
endif()

# ROS
find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	sensor_msgs
	message_generation
	image_transport
	cv_bridge)
add_service_files(
	FILES
	Scan.srv
)
generate_messages(
	DEPENDENCIES
	std_msgs
	sensor_msgs
)
catkin_package(CATKIN_DEPENDS
	roscpp
	rospy
	std_msgs
	sensor_msgs
	image_transport
	cv_bridge)
include_directories(${catkin_INCLUDE_DIRS})

# Camera calibration node
add_executable(camera_calibration nodes/camera_calibration.cpp)
target_link_libraries(camera_calibration ${catkin_LIBRARIES} coro_eyes_sdk)

# Adjust focus node
add_executable(adjust_focus nodes/adjust_focus.cpp)
target_link_libraries(adjust_focus ${catkin_LIBRARIES} coro_eyes_sdk)

# Upload patterns node
add_executable(upload_patterns nodes/upload_patterns.cpp)
target_link_libraries(upload_patterns ${catkin_LIBRARIES} coro_eyes_sdk)

# Scan node
add_executable(scan nodes/scan.cpp)
target_link_libraries(scan ${catkin_LIBRARIES} coro_eyes_sdk)

# Install cpp nodes
install(TARGETS camera_calibration adjust_focus upload_patterns scan DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# Install python nodes
catkin_install_python(PROGRAMS scripts/coro_eyes.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

