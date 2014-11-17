cmake_minimum_required(VERSION 2.8.3)
project(uvc_cam)
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs image_transport tf camera_info_manager dynamic_reconfigure driver_base)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

#add_definitions("-D_FILE_OFFSET_BITS=64") #No idea what this is for... maybe something for the SDL library?

# auto-generated dynamic reconfiguration GUI
generate_dynamic_reconfigure_options(cfg/UVCCam.cfg)

#Build the uvc_cam library
add_library(uvc_cam src/uvc_cam/uvc_cam.cpp)

# librt is needed for clock_gettime();  not sure how this interacts with
# real-time....
#target_link_libraries(dc1394cam rt)

#add_executable(view src/view/view.cpp)
#target_link_libraries(view uvc_cam)
#target_link_libraries(view ${SDL_LIBRARIES})

#add_executable(enumerate src/enumerate/enumerate.cpp)
#target_link_libraries(enumerate uvc_cam)

#add_executable(dump src/dump/dump.cpp src/dump/avilib.c)
#target_link_libraries(dump uvc_cam)

#add_executable(play src/play.cpp)
#target_link_libraries(play uvc_cam)

#add_executable(sender src/sender.cpp)
#target_link_libraries(sender uvc_cam)

add_executable(uvc_cam_node src/uvc_cam_node.cpp)
target_link_libraries(uvc_cam_node uvc_cam ${catkin_LIBRARIES})
add_dependencies(uvc_cam_node uvc_cam_gencfg)

#add_executable(senderStereo src/stereo_sender.cpp)
#target_link_libraries(senderStereo uvc_cam)

#add_executable(dump_send src/dump/dump_send.cpp src/dump/avilib.c)
#target_link_libraries(dump_send uvc_cam)

catkin_package(
  DEPENDS
  CATKIN_DEPENDS roscpp sensor_msgs image_transport tf camera_info_manager dynamic_reconfigure driver_base
  INCLUDE_DIRS include
  LIBRARIES uvc_cam
)

install(TARGETS uvc_cam_node uvc_cam
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  )

install(FILES test_uvc.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
