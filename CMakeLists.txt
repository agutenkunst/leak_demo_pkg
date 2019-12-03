cmake_minimum_required(VERSION 2.8.3)
project(leak_demo_pkg)

find_package(catkin REQUIRED COMPONENTS pluginlib)

catkin_package()


###########
## Build ##
###########


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(plugin src/plugin.cpp)

add_executable(main src/main.cpp)

target_link_libraries(main ${catkin_LIBRARIES})

#############
## Install ##
#############

install(TARGETS plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(FILES
  plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)