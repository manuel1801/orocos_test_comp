
cmake_minimum_required(VERSION 2.8.3)
 
project(orocos_test_comp)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set (CMAKE_CXX_STANDARD 11)
set(CMAKE_CONFIGURATION_TYPES "Release" CACHE STRING "" FORCE)
SET(CMAKE_CXX_FLAGS_DISTRIBUTION "-O3")
set(CMAKE_POSITION_INDEPENDENT_CODE ON) 

include_directories(${PROJECT_SOURCE_DIR}/include)

find_package(OROCOS-RTT REQUIRED ${RTT_HINTS})
if (NOT OROCOS-RTT_FOUND)
  message (FATAL_ERROR "\nCould not find Orocos. Please use the shell command\n 'source orocos_toolchain/env.sh' and then run cmake again.")
endif()

find_package(RST-RT REQUIRED)
IF (RST-RT_FOUND)
	message(STATUS "RST-RT found. Version: ${RST-RT_VERSION}")
	include_directories(BEFORE SYSTEM ${RST-RT_INCLUDE_DIRS})
	add_definitions(${RST-RT_CFLAGS})
	link_directories(${RST-RT_LIBRARY_DIRS})
ENDIF()

find_package(orocos_kdl REQUIRED)
IF (orocos_kdl_FOUND)
    message(STATUS "orocos_kdl found. Version: ${orocos_kdl_VERSION}")
    include_directories(BEFORE SYSTEM ${orocos_kdl_INCLUDE_DIRS})
    add_definitions(${orocos_kdl_CFLAGS})
    link_directories(${orocos_kdl_LIBRARY_DIRS})
ENDIF()


find_library(sioclient sioclient)
find_package(Boost REQUIRED COMPONENTS system)
IF (Boost_FOUND)
        message(STATUS "Boost found. Version: ${Boost_VERSION}")
        include_directories(BEFORE SYSTEM ${Boost_INCLUDE_DIRS})
        add_definitions(${Boost_CFLAGS})
        link_directories(${Boost_LIBRARY_DIR})
ENDIF()

find_package(Eigen3)
IF (Eigen3_FOUND)
    message(STATUS "Eigen3 found. Version: ${Eigen3_VERSION}")
    include_directories(BEFORE SYSTEM ${Eigen3_INCLUDE_DIRS})
    add_definitions(${Eigen3_CFLAGS})
    link_directories(${Eigen3_LIBRARY_DIRS})
ENDIF()

find_package(urdf REQUIRED)

find_package(kdl_parser REQUIRED)
message(STATUS "kdl_parser_INCLUDE_DIRS: ${kdl_parser_INCLUDE_DIRS}")
include_directories(${kdl_parser_INCLUDE_DIRS})

find_library(trac_ik trac_ik)


find_library(ref reflexxes)
IF (ref)
    message(STATUS "Reflexxes found")
ENDIF()

find_package(PkgConfig REQUIRED)

find_path(LIBNLOPT_INCLUDE_DIRS
          NAMES nlopt.h
          HINTS /usr/local/include)
find_library(LIBNLOPT_LIBRARIES
             NAMES nlopt
             HINTS /usr/lib/ /usr/x86_64-linux-gnu/
             PATH_SUFFIXES lib)

include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)

orocos_component(${CMAKE_PROJECT_NAME} 	${PROJECT_SOURCE_DIR}/include/orocos_test_comp.hpp 
										${PROJECT_SOURCE_DIR}/src/orocos_test_comp.cpp) 

target_link_libraries(${CMAKE_PROJECT_NAME} ${RST-RT_LIBRARIES} ${orocos_kdl_LIBRARIES} ${ref}
${Boost_LIBRARIES} ${urdfdom_LIBRARIES} ${Eigen3_LIBRARIES} ${LIBNLOPT_LIBRARIES} ${trac_ik} ${urdf} ${PkgConfig} ${kdl_parser} ${LIBNLOPT_LIBRARIES})
orocos_generate_package()
