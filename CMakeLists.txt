cmake_minimum_required(VERSION 2.8.3)
project(vp100_ros)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#add_definitions(-std=c++11) # Use C++11

find_package(catkin REQUIRED COMPONENTS
  rosconsole
  roscpp
  sensor_msgs
)

#add_subdirectory(sdk)

set(SDK_PATH "./sdk/")

FILE(GLOB SDK_SRC 
  "${SDK_PATH}/src/impl/src/serial/*.cpp"
  "${SDK_PATH}/src/nvilidar/*.cpp"
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_SOURCE_DIR}/sdk/src/nvilidar
  ${PROJECT_SOURCE_DIR}/sdk/src/impl/include
  ${PROJECT_SOURCE_DIR}/sdk/src/3rdparty/include
)

add_executable(vp100_node src/vp100_node.cpp  ${SDK_SRC})
add_executable(vp100_client src/vp100_client.cpp)

target_link_libraries(vp100_node
   ${catkin_LIBRARIES} 
 )
target_link_libraries(vp100_client
   ${catkin_LIBRARIES} 
 )

install(TARGETS vp100_node vp100_client
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(DIRECTORY launch startup sdk
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)


