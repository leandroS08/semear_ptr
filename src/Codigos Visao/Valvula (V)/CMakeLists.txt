cmake_minimum_required(VERSION 2.8)
project( valvula ) 
find_package( OpenCV REQUIRED )
add_executable( valvula code.cpp )
add_executable( testeCor codeCor.cpp )
target_link_libraries( valvula ${OpenCV_LIBS} )
target_link_libraries( testeCor ${OpenCV_LIBS} )

