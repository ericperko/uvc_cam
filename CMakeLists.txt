cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)

set(ROS_BUILD_TYPE Debug)

rospack(uvc_cam)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR})

pkg_check_modules(SDL REQUIRED sdl)
include_directories(${SDL_INCLUDE_DIRS})
link_directories(${SDL_LIBRARY_DIRS})

rospack_add_library(uvc_cam src/uvc_cam/uvc_cam.cpp)
# librt is needed for clock_gettime();  not sure how this interacts with
# real-time....
#target_link_libraries(dc1394cam rt)

#rospack_add_executable(cam_dump src/util/cam_dump/cam_dump.cpp)
#target_link_libraries(cam_dump dc1394cam)

rospack_add_executable(view src/view/view.cpp)
target_link_libraries(view uvc_cam)
target_link_libraries(view ${SDL_LIBRARIES})

rospack_add_executable(enumerate src/enumerate/enumerate.cpp)
target_link_libraries(enumerate uvc_cam)

rospack_add_executable(dump src/dump/dump.cpp src/dump/avilib.c)
target_link_libraries(dump uvc_cam)
