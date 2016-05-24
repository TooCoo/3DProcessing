cmake_minimum_required(VERSION 3.3)
project(3d_processing)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

######################################################################
# LIBRARIES
######################################################################

# Spectra
set(SPECTRA_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib)
include_directories(${SPECTRA_INCLUDE_DIR})

# Eigen
set(EIGEN3_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib/eigen3 )
include_directories(${EIGEN3_INCLUDE_DIR})

# FreeGLUT
set(FreeGLUT_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib/freeglut-3.0.0 )
include_directories(${FreeGLUT_INCLUDE_DIR})

## ANN
set(ANN_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib/ann_1.1.2 )
include_directories(${ANN_INCLUDE_DIR})

# GLFW
set(GLFW_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib/glfw-3.1.2)
add_subdirectory(${GLFW_DIR})
include_directories(${GLFW_DIR}/include)

# OpenMesh - not sure how to make this work ATM
#set(OpenMesh_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib/OpenMesh-6.0)
#add_subdirectory(${OpenMesh_DIR})
#include_directories(${OpenMesh_DIR}/include)


######################################################################
# BUILD EXECUTABLES
######################################################################

find_library(ANN NAMES "libANN.a" PATHS ${ANN_INCLUDE_DIR}/lib)

find_package(GLUT)

# source
set(SOURCE_FILES main.cpp)
add_executable(3d_processing ${SOURCE_FILES})

target_link_libraries(3d_processing glfw ${ANN} ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} ${GLFW_LIBRARIES} OpenMeshCore OpenMeshTools)

