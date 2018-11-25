cmake_minimum_required(VERSION 2.8)
project( cone ) 
find_package( OpenCV REQUIRED )
add_executable( cone code.cpp )
add_executable( testeCor codeCor.cpp )
add_executable( v2 code2.cpp )
target_link_libraries( cone ${OpenCV_LIBS} )
target_link_libraries( testeCor ${OpenCV_LIBS} )
target_link_libraries( v2 ${OpenCV_LIBS} )


