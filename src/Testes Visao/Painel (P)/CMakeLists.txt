cmake_minimum_required(VERSION 2.8)
project( Painel ) 
find_package( OpenCV REQUIRED )
add_executable( Painel code.cpp )
target_link_libraries( Painel ${OpenCV_LIBS} )

