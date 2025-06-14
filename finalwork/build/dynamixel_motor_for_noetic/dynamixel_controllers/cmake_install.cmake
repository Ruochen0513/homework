# Install script for directory: /home/amadeus/forTurtle/src/dynamixel_motor_for_noetic/dynamixel_controllers

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/amadeus/forTurtle/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
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

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dynamixel_controllers/srv" TYPE FILE FILES
    "/home/amadeus/forTurtle/src/dynamixel_motor_for_noetic/dynamixel_controllers/srv/RestartController.srv"
    "/home/amadeus/forTurtle/src/dynamixel_motor_for_noetic/dynamixel_controllers/srv/SetComplianceMargin.srv"
    "/home/amadeus/forTurtle/src/dynamixel_motor_for_noetic/dynamixel_controllers/srv/SetCompliancePunch.srv"
    "/home/amadeus/forTurtle/src/dynamixel_motor_for_noetic/dynamixel_controllers/srv/SetComplianceSlope.srv"
    "/home/amadeus/forTurtle/src/dynamixel_motor_for_noetic/dynamixel_controllers/srv/SetSpeed.srv"
    "/home/amadeus/forTurtle/src/dynamixel_motor_for_noetic/dynamixel_controllers/srv/SetTorqueLimit.srv"
    "/home/amadeus/forTurtle/src/dynamixel_motor_for_noetic/dynamixel_controllers/srv/StartController.srv"
    "/home/amadeus/forTurtle/src/dynamixel_motor_for_noetic/dynamixel_controllers/srv/StopController.srv"
    "/home/amadeus/forTurtle/src/dynamixel_motor_for_noetic/dynamixel_controllers/srv/TorqueEnable.srv"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/amadeus/forTurtle/build/dynamixel_motor_for_noetic/dynamixel_controllers/catkin_generated/safe_execute_install.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dynamixel_controllers/cmake" TYPE FILE FILES "/home/amadeus/forTurtle/build/dynamixel_motor_for_noetic/dynamixel_controllers/catkin_generated/installspace/dynamixel_controllers-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/amadeus/forTurtle/devel/include/dynamixel_controllers")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/amadeus/forTurtle/devel/share/roseus/ros/dynamixel_controllers")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/amadeus/forTurtle/devel/share/common-lisp/ros/dynamixel_controllers")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/amadeus/forTurtle/devel/share/gennodejs/ros/dynamixel_controllers")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/amadeus/forTurtle/devel/lib/python3.8/site-packages/dynamixel_controllers")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3.8/site-packages" TYPE DIRECTORY FILES "/home/amadeus/forTurtle/devel/lib/python3.8/site-packages/dynamixel_controllers" REGEX "/\\_\\_init\\_\\_\\.py$" EXCLUDE REGEX "/\\_\\_init\\_\\_\\.pyc$" EXCLUDE)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3.8/site-packages" TYPE DIRECTORY FILES "/home/amadeus/forTurtle/devel/lib/python3.8/site-packages/dynamixel_controllers" FILES_MATCHING REGEX "/home/amadeus/forTurtle/devel/lib/python3\\.8/site-packages/dynamixel_controllers/.+/__init__.pyc?$")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/amadeus/forTurtle/build/dynamixel_motor_for_noetic/dynamixel_controllers/catkin_generated/installspace/dynamixel_controllers.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dynamixel_controllers/cmake" TYPE FILE FILES "/home/amadeus/forTurtle/build/dynamixel_motor_for_noetic/dynamixel_controllers/catkin_generated/installspace/dynamixel_controllers-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dynamixel_controllers/cmake" TYPE FILE FILES
    "/home/amadeus/forTurtle/build/dynamixel_motor_for_noetic/dynamixel_controllers/catkin_generated/installspace/dynamixel_controllersConfig.cmake"
    "/home/amadeus/forTurtle/build/dynamixel_motor_for_noetic/dynamixel_controllers/catkin_generated/installspace/dynamixel_controllersConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dynamixel_controllers" TYPE FILE FILES "/home/amadeus/forTurtle/src/dynamixel_motor_for_noetic/dynamixel_controllers/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dynamixel_controllers" TYPE DIRECTORY FILES "/home/amadeus/forTurtle/src/dynamixel_motor_for_noetic/dynamixel_controllers/launch")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/dynamixel_controllers" TYPE PROGRAM FILES
    "/home/amadeus/forTurtle/src/dynamixel_motor_for_noetic/dynamixel_controllers/nodes/controller_manager.py"
    "/home/amadeus/forTurtle/src/dynamixel_motor_for_noetic/dynamixel_controllers/nodes/controller_spawner.py"
    )
endif()

