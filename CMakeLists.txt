cmake_minimum_required(VERSION 2.8)
project( stereo3Dcalculation )

find_package( OpenCV REQUIRED )
add_executable( stereo3Dcalculation main.cpp images/ )

include_directories( images/ )

target_link_libraries( stereo3Dcalculation ${OpenCV_LIBS} )