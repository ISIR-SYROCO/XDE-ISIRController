cmake_minimum_required(VERSION 2.8)

SET(PROJECT_NAME ISIRControllerThreadXDE)
project(${PROJECT_NAME} CXX)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-fpermissive")

if(MSVC)
    set(LIB_TYPE "STATIC")
    set(suffix_type "_static")
    set(install_type "ARCHIVE")
else()
    set(LIB_TYPE "SHARED")
    set(suffix_type "")
    if(UNIX)
        set(install_type "LIBRARY")
    else()
        set(install_type "RUNTIME")
    endif()
endif()

IF (UNIX)
    ADD_DEFINITIONS(-DUNIX)
ENDIF (UNIX)

# Check dependencies
find_package(PkgConfig REQUIRED)

find_package(Orocos-RTT REQUIRED ${RTT_HINTS})

PKG_CHECK_MODULES(PC_OCL_DEPLOY ocl-deployment-gnulinux REQUIRED)
PKG_CHECK_MODULES(PC_OCL orocos-ocl-gnulinux REQUIRED)
PKG_CHECK_MODULES(PC_XDECORE xdecore REQUIRED)
if(NOT PC_XDECORE_FOUND)
    message("Failed to detect xdecore, check PKG_CONFIG_PATH." )
endif(NOT PC_XDECORE_FOUND)

PKG_CHECK_MODULES(PC_PYTHON2 python2 REQUIRED)

#PKG_CHECK_MODULES(PC_EIGEN eigen3 REQUIRED)
#if(NOT PC_EIGEN_FOUND)
#    message("Failed to detect Eigen, check PKG_CONFIG_PATH." )
#endif(NOT PC_EIGEN_FOUND)
PKG_CHECK_MODULES(PC_EIGEN eigen_lgsm REQUIRED)
if(NOT PC_EIGEN_FOUND)
    message("Failed to detect Eigen, check PKG_CONFIG_PATH." )
endif(NOT PC_EIGEN_FOUND)

find_package(PkgConfig REQUIRED)
PKG_CHECK_MODULES(PC_WLQP_CONTROL wLQP-Control REQUIRED)
PKG_CHECK_MODULES(PC_OCRA_FRAMEWORK ocra_framework REQUIRED)

#PKG_CHECK_MODULES(PC_ORCISIR_CONTROLLER orcisir_ISIRController REQUIRED)

ADD_DEFINITIONS(-DOROCOS_TARGET=gnulinux)

FILE(GLOB_RECURSE folder_source ./src/*.cpp)

SOURCE_GROUP("Source Files" FILES ${folder_source})

# Defines the orocos_* cmake macros. See that file for additional
# documentation.
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)

include_directories(${PROJECT_SOURCE_DIR}/include)

include_directories(${PC_EIGEN_INCLUDE_DIRS})
#include_directories(/usr/include/eigen3)
#include_directories(/usr/include/eigen3/unsupported)

include_directories(
    ${OROCOS-RTT_INCLUDE_DIR}
    ${PC_XDECORE_INCLUDE_DIRS}   
    ${PC_OCL_INCLUDE_DIRS}
    ${PC_OCL_DEPLOY_INCLUDE_DIRS}
    ${PC_WLQP_CONTROL_INCLUDE_DIRS}
    ${PC_OCRA_FRAMEWORK_INCLUDE_DIRS} 
    ${PC_PYTHON2_INCLUDE_DIRS}
)

LINK_DIRECTORIES(
    ${PC_XDECORE_LIBRARY_DIRS}
    ${PC_OCL_LIBRARY_DIRS}
    ${PC_OCL_DEPLOY_LIBRARY_DIRS}
    ${PC_WLQP_CONTROL_LIBRARY_DIRS}
    ${PC_OCRA_FRAMEWORK_LIBRARY_DIRS}
    ${PC_PYTHON2_LIBRARY_DIRS}
)

orocos_component(${PROJECT_NAME}
    ${folder_source})

target_link_libraries(
    ${PROJECT_NAME}
    ${Orocos-RTT_LIBRARIES}
    ${PC_OCL_LIBRARIES}
    ${PC_OCL_DEPLOY_LIBRARIES}
    ${PC_XDECORE_LIBRARIES}
    ${PC_WLQP_CONTROL_LIBRARIES}
    ${PC_OCRA_FRAMEWORK_LIBRARIES}
    ${PC_PYTHON2_LIBRARIES}
)
#add_subdirectory(src)
