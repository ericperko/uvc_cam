cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)

set(ROS_BUILD_TYPE Release)

rosbuild_init(uvc_cam)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR})

pkg_check_modules(SDL REQUIRED sdl)
include_directories(${SDL_INCLUDE_DIRS})
link_directories(${SDL_LIBRARY_DIRS})
add_definitions("-D_FILE_OFFSET_BITS=64")

rosbuild_add_library(uvc_cam src/uvc_cam/uvc_cam.cpp)
# librt is needed for clock_gettime();  not sure how this interacts with
# real-time....
#target_link_libraries(dc1394cam rt)

rosbuild_add_executable(view src/view/view.cpp)
target_link_libraries(view uvc_cam)
target_link_libraries(view ${SDL_LIBRARIES})

rosbuild_add_executable(enumerate src/enumerate/enumerate.cpp)
target_link_libraries(enumerate uvc_cam)

rosbuild_add_executable(dump src/dump/dump.cpp src/dump/avilib.c)
target_link_libraries(dump uvc_cam)

rosbuild_add_executable(play src/play.cpp)
target_link_libraries(play uvc_cam)

rosbuild_add_executable(sender src/sender.cpp)
target_link_libraries(sender uvc_cam)

rosbuild_add_executable(senderIT src/senderIT.cpp)
target_link_libraries(senderIT uvc_cam)

rosbuild_add_executable(senderStereo src/stereo_sender.cpp)
target_link_libraries(senderStereo uvc_cam)

rosbuild_add_executable(record_theora src/record_theora.cpp)
target_link_libraries(record_theora theora theoraenc theoradec uvc_cam)

rosbuild_add_executable(dump_send src/dump/dump_send.cpp src/dump/avilib.c)
target_link_libraries(dump_send uvc_cam)
