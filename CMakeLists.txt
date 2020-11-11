cmake_minimum_required(VERSION 2.8.3)
project(davis_corner)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  cv_bridge
  image_transport
  sensor_msgs
  dvs_msgs
  message_generation
  common_srv
)

find_package(OpenCV REQUIRED)


add_message_files(FILES
    Corners.msg
)

generate_messages(
 DEPENDENCIES
 std_msgs
)



catkin_package(
INCLUDE_DIRS LIBRARIES rosopencv_t CATKIN_DEPENDS common_srv cv_bridge image_transport roscpp sensor_msgs CATKIN_DEPENDS message_runtime std_msgs )

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
)

FILE(GLOB SOURCE_FILES *.hpp *.cpp *.h
        ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
        )

add_executable(ball_detector_node src/main.cpp ${SOURCE_FILES})

target_link_libraries(ball_detector_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})










