cmake_minimum_required(VERSION 3.0.2)
project(leptrino_force_torque_ros_driver)

find_package(catkin REQUIRED COMPONENTS geometry_msgs roscpp std_srvs)

catkin_package()

add_library(leptrino_sensor STATIC src/protocol.cpp src/serial_port.cpp src/sensor.cpp)
target_include_directories(leptrino_sensor PUBLIC include)
target_compile_features(leptrino_sensor PUBLIC cxx_std_11)

add_executable(leptrino_force_torque_ros_driver src/node.cpp)
target_include_directories(leptrino_force_torque_ros_driver PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(leptrino_force_torque_ros_driver leptrino_sensor ${catkin_LIBRARIES})

install(TARGETS leptrino_force_torque_ros_driver RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES README.md LICENSE DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(Threads REQUIRED)
  foreach(test core sensor publication zero_wrench output_rotation)
    add_executable(${PROJECT_NAME}_${test}_test test/${test}_test.cpp)
    target_compile_options(${PROJECT_NAME}_${test}_test PRIVATE -UNDEBUG)
    target_link_libraries(${PROJECT_NAME}_${test}_test leptrino_sensor Threads::Threads util)
    add_test(NAME ${PROJECT_NAME}_${test} COMMAND ${PROJECT_NAME}_${test}_test)
    set_tests_properties(${PROJECT_NAME}_${test} PROPERTIES TIMEOUT 10)
  endforeach()
endif()
