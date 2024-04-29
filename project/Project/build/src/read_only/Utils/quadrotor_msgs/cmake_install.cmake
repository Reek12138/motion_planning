# Install script for directory: /home/reek/motion_planning/project/Project/src/src/read_only/Utils/quadrotor_msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/reek/motion_planning/project/Project/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/quadrotor_msgs/msg" TYPE FILE FILES
    "/home/reek/motion_planning/project/Project/src/src/read_only/Utils/quadrotor_msgs/msg/AuxCommand.msg"
    "/home/reek/motion_planning/project/Project/src/src/read_only/Utils/quadrotor_msgs/msg/Corrections.msg"
    "/home/reek/motion_planning/project/Project/src/src/read_only/Utils/quadrotor_msgs/msg/Gains.msg"
    "/home/reek/motion_planning/project/Project/src/src/read_only/Utils/quadrotor_msgs/msg/OutputData.msg"
    "/home/reek/motion_planning/project/Project/src/src/read_only/Utils/quadrotor_msgs/msg/PositionCommand.msg"
    "/home/reek/motion_planning/project/Project/src/src/read_only/Utils/quadrotor_msgs/msg/PPROutputData.msg"
    "/home/reek/motion_planning/project/Project/src/src/read_only/Utils/quadrotor_msgs/msg/Serial.msg"
    "/home/reek/motion_planning/project/Project/src/src/read_only/Utils/quadrotor_msgs/msg/SO3Command.msg"
    "/home/reek/motion_planning/project/Project/src/src/read_only/Utils/quadrotor_msgs/msg/StatusData.msg"
    "/home/reek/motion_planning/project/Project/src/src/read_only/Utils/quadrotor_msgs/msg/TRPYCommand.msg"
    "/home/reek/motion_planning/project/Project/src/src/read_only/Utils/quadrotor_msgs/msg/Odometry.msg"
    "/home/reek/motion_planning/project/Project/src/src/read_only/Utils/quadrotor_msgs/msg/PolynomialTrajectory.msg"
    "/home/reek/motion_planning/project/Project/src/src/read_only/Utils/quadrotor_msgs/msg/LQRTrajectory.msg"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/quadrotor_msgs/cmake" TYPE FILE FILES "/home/reek/motion_planning/project/Project/build/src/read_only/Utils/quadrotor_msgs/catkin_generated/installspace/quadrotor_msgs-msg-paths.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/reek/motion_planning/project/Project/devel/include/quadrotor_msgs")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/reek/motion_planning/project/Project/devel/share/roseus/ros/quadrotor_msgs")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/reek/motion_planning/project/Project/devel/share/common-lisp/ros/quadrotor_msgs")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/reek/motion_planning/project/Project/devel/share/gennodejs/ros/quadrotor_msgs")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/reek/motion_planning/project/Project/devel/lib/python3/dist-packages/quadrotor_msgs")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/reek/motion_planning/project/Project/devel/lib/python3/dist-packages/quadrotor_msgs")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/reek/motion_planning/project/Project/build/src/read_only/Utils/quadrotor_msgs/catkin_generated/installspace/quadrotor_msgs.pc")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/quadrotor_msgs/cmake" TYPE FILE FILES "/home/reek/motion_planning/project/Project/build/src/read_only/Utils/quadrotor_msgs/catkin_generated/installspace/quadrotor_msgs-msg-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/quadrotor_msgs/cmake" TYPE FILE FILES
    "/home/reek/motion_planning/project/Project/build/src/read_only/Utils/quadrotor_msgs/catkin_generated/installspace/quadrotor_msgsConfig.cmake"
    "/home/reek/motion_planning/project/Project/build/src/read_only/Utils/quadrotor_msgs/catkin_generated/installspace/quadrotor_msgsConfig-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/quadrotor_msgs" TYPE FILE FILES "/home/reek/motion_planning/project/Project/src/src/read_only/Utils/quadrotor_msgs/package.xml")
endif()

