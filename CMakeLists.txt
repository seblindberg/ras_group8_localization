cmake_minimum_required(VERSION 2.8.3)
project(ras_group8_localization)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  COMPONENTS
  roscpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf
  ras_group8_map
)

catkin_package(
  INCLUDE_DIRS
    include
    
  LIBRARIES
    ${PROJECT_NAME}_particle_filter

  CATKIN_DEPENDS
    roscpp
    nav_msgs
    sensor_msgs
    geometry_msgs
    tf
    ras_group8_map

# DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#
# Particle Filter
#

add_library(${PROJECT_NAME}_particle_filter
  src/ParticleFilter.cpp
)

add_dependencies(${PROJECT_NAME}_particle_filter
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_particle_filter
  ${catkin_LIBRARIES}
)

if (CATKIN_ENABLE_TESTING)
  catkin_add_gtest(${PROJECT_NAME}_particle_filter_tests
    test/test_particle_filter.cpp
  )

  target_link_libraries(${PROJECT_NAME}_particle_filter_tests
    ${PROJECT_NAME}_particle_filter
    ras_group8_map_grid
    ${catkin_LIBRARIES}
  )
endif()

add_executable(${PROJECT_NAME}
  src/localization_node.cpp
  src/Localization.cpp
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_particle_filter
  ${catkin_LIBRARIES}
)

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED ras_group8_util)
  
  add_rostest_gtest(${PROJECT_NAME}_tests
    test/test_localization.test
    test/test_localization.cpp
  )

  target_link_libraries(${PROJECT_NAME}_tests
    ras_group8_map_grid
    ras_group8_util_bmp
    ${catkin_LIBRARIES}
  )
endif()



#catkin_add_gtest(localization_tests test/test_localization.cpp src/Localization.cpp)

# Make main executable
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  TARGETS ${PROJECT_NAME}_particle_filter
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install headers
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)