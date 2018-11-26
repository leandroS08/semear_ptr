cmake_minimum_required(VERSION 2.8)

project( qr_code ) 

set(CMAKE_MODULE_PATH ${ZBARCV_SOURCE_DIR})

find_package( OpenCV REQUIRED )
find_package (ZBar0)

add_executable( qr_code main.cpp )

target_link_libraries( qr_code ${OpenCV_LIBS} ${ZBAR_LIBRARIES})

