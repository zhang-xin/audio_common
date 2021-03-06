cmake_minimum_required(VERSION 2.8.3)

project(audio_capture)

find_package(catkin REQUIRED COMPONENTS roscpp audio_common_msgs)

find_package(PkgConfig)
pkg_check_modules(GST1.0 gstreamer-1.0 REQUIRED)
pkg_check_modules(GSTAUDIO1.0 gstreamer-audio-1.0 REQUIRED)

find_package(Boost REQUIRED COMPONENTS thread)

include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${GST1.0_INCLUDE_DIRS} ${GSTAUDIO1.0_INCLUDE_DIRS})

add_compile_options(-std=c++11 -Wall)

catkin_package()

add_executable(audio_capture src/audio_capture.cpp)
target_link_libraries(audio_capture ${catkin_LIBRARIES} ${GST1.0_LIBRARIES} ${GSTAUDIO1.0_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(audio_capture ${catkin_EXPORTED_TARGETS})

install(TARGETS audio_capture
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
