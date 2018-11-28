cmake_minimum_required(VERSION 2.8)
project( gravar ) 
find_package( OpenCV REQUIRED )
add_executable( gravar tamplete.cpp )
target_link_libraries( gravar ${OpenCV_LIBS} )

