cmake_minimum_required(VERSION 2.8.3)
project(rosgraph_monitor)

find_package(catkin REQUIRED COMPONENTS
    rospy
)

catkin_python_setup()

catkin_package()

# INSTALL
catkin_install_python(PROGRAMS 
  scripts/monitor
  scripts/rossystem_snapshot
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY resources
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
