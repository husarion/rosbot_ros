cmake_minimum_required(VERSION 2.8.3)
project(rosbot_navigation)

find_package(catkin REQUIRED)

catkin_package(
)

install(DIRECTORY launch
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


install(DIRECTORY maps
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
