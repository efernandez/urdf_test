cmake_minimum_required(VERSION 2.8.3)
project(urdf_test)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  urdf
)

catkin_package()

include_directories(SYSTEM ${catkin_INCLUDE_DIRS})

add_executable(urdf_test src/urdf_test.cpp)
target_link_libraries(urdf_test ${catkin_LIBRARIES})

install(TARGETS urdf_test
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
