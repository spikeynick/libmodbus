cmake_minimum_required(VERSION 2.8)
project(libmodbus)

set(CMAKE_DEBUG_POSTFIX "d" CACHE STRING "Set Debug library postfix" )


include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)
    
set(LIBMODBUS_SOURCES
src/modbus.cpp
src/modbus-data.c
src/modbus-rtu.cpp
src/util.cpp
src/modbus-data.c
)

set(LIBMODBUS_HEADERS
src/modbus.hpp
src/modbus.h
src/modbus-private.h
src/modbus-rtu.hpp
src/modbus-version.h
)

add_library(libmodbus ${LIBMODBUS_SOURCES} ${LIBMODBUS_HEADERS} )
