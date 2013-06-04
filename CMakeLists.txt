# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(terse_roscpp)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS roscpp)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
add_library(${PROJECT_NAME} src/params.cpp)

catkin_package(
    DEPENDS roscpp
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES terse_roscpp
)
