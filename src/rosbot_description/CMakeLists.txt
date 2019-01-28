cmake_minimum_required(VERSION 2.8.3)
project(rosbot_description)

find_package(catkin REQUIRED
	rospy
)

catkin_python_setup()

catkin_package(
)

catkin_install_python(PROGRAMS
  scripts/rosbot_description/serial_bridge.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rosbot_description
)

install(DIRECTORY launch
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY meshes
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


install(DIRECTORY rviz
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY urdf
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)