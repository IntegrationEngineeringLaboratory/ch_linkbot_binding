cmake_minimum_required(VERSION 3.2.1)
project(c-stem-robotics-library CXX C)

set(SOURCES
	position.hpp
	robotAction.hpp
	robot_utilities.hpp
	robot_position_recorder.cpp
	)

add_library(c_stem_robotics ${SOURCES})
