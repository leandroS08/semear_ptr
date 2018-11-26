cmake_minimum_required(VERSION 2.8)
project( qr_code ) 
find_package( OpenCV REQUIRED )
add_executable( qr_code main.cpp )
target_link_libraries( qr_code ${OpenCV_LIBS} qrReader.h)

