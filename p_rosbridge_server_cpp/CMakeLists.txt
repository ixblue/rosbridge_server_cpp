cmake_minimum_required(VERSION 3.1)
project(p_rosbridge_server_cpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Werror=return-type)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(CATKIN_DEPS
    librosqt
    ros_babel_fish
    roscpp
    std_msgs
    rosbridge_cpp_msgs
    )
find_package(catkin REQUIRED
    ${CATKIN_DEPS}
    )

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
find_package(Qt5 COMPONENTS Core Network WebSockets REQUIRED)

if(CATKIN_ENABLE_TESTING AND ENABLE_COVERAGE_TESTING)
  find_package(code_coverage REQUIRED)
  # Add compiler flags for coverage instrumentation before defining any targets
  APPEND_COVERAGE_COMPILER_FLAGS()
endif()


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES p_rosbridge_server_cpp
    CATKIN_DEPENDS ${CATKIN_DEPS}
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  src
)

## Declare a C++ library
add_library(${PROJECT_NAME}
    src/ROSNode.cpp
    src/WSClient.cpp
    src/ServiceCallerWithTimeout.cpp
    src/rosbridge_protocol.cpp
    src/ros_to_nlohmann.cpp
    src/nlohmann_to_ros.cpp
    src/utils/NodeWatchdog.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  Qt5::Core
  Qt5::Network
  Qt5::WebSockets
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node
  src/p_rosbridge_server_cpp_node.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  Qt5::Core
  Qt5::Network
  Qt5::WebSockets
  ${PROJECT_NAME}
)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_custom_target(readme SOURCES README.md)

file(GLOB_RECURSE LAUNCH_FILES "launch/*")
add_custom_target(launch_files SOURCES ${LAUNCH_FILES})

file(GLOB_RECURSE PYTHON_FILES "test/*.py")
add_custom_target(python_test_files SOURCES ${PYTHON_FILES})

file(GLOB_RECURSE ROSTEST_FILES "test/*.test")
add_custom_target(rostest_files SOURCES ${ROSTEST_FILES})

#############
## Install ##
#############

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
install(TARGETS ${PROJECT_NAME}_node ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# launch files
install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

if (CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED
    ${CATKIN_DEPS}
    sensor_msgs
  )


  # Add gtest based cpp test target and link libraries
  catkin_add_gtest(ROSJsonConverterTest test/test_ros_json_converter.cpp)
  if(TARGET ROSJsonConverterTest)
    target_link_libraries(ROSJsonConverterTest ${PROJECT_NAME} ${catkin_LIBRARIES})
  endif()

  find_package(rostest REQUIRED)
  add_rostest(test/test_cbor_raw.test)
  add_rostest(test/test_json_pub.test)

  find_package(Qt5Test REQUIRED)

  add_rostest_gtest(bridge_tester test/bridge_tester.test test/BridgeTester.cpp)
  #catkin_add_gtest(bridge_tester test/BridgeTester.cpp)
  target_link_libraries(bridge_tester ${PROJECT_NAME} ${catkin_LIBRARIES} Qt5::Test)

  # Benchmark tests
  add_executable(benchmark_gps_pub test/benchmarks/benchmark_gps_pub.cpp)
  target_link_libraries(benchmark_gps_pub ${catkin_LIBRARIES})

  add_executable(benchmark_image_pub test/benchmarks/benchmark_image_pub.cpp)
  target_link_libraries(benchmark_image_pub ${catkin_LIBRARIES})

  ## Add folders to be run by python nosetests
  # catkin_add_nosetests(test)

  if(ENABLE_COVERAGE_TESTING)
    set(COVERAGE_EXCLUDES "*/${PROJECT_NAME}/test*" "*/${PROJECT_NAME}/src/nlohmann*" "*/${PROJECT_NAME}_autogen/*" "*/bridge_tester_autogen/*")
    add_code_coverage(
      NAME ${PROJECT_NAME}_coverage_report
      DEPENDENCIES ROSJsonConverterTest bridge_tester
    )
  endif()
endif ()
