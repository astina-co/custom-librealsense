# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2019 Intel Corporation. All Rights Reserved.
#  minimum required cmake version: 3.1.0
cmake_minimum_required(VERSION 3.1.0)

project(rs-astina)

# configuration
set(PRECOMPILED_LIBS "C:/Workspace/third-party")

add_definitions(-D_ENABLE_EXTENDED_ALIGNED_STORAGE)
string(REPLACE "/EHsc" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /EHa")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP")

# open3d
find_package(Open3D REQUIRED HINTS ${PRECOMPILED_LIBS}/Open3D)
link_directories(${Open3D_LIBRARY_DIRS})
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${Open3D_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Open3D_CXX_FLAGS}")

# opencv
list(APPEND CMAKE_MODULE_PATH ${PRECOMPILED_LIBS}/OpenCV)
find_package(OpenCV REQUIRED HINTS ${PRECOMPILED_LIBS}/OpenCV)

# fmt
find_package(fmt REQUIRED HINTS ${PRECOMPILED_LIBS}/fmt)

add_subdirectory(third-party)

# json
set(json_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third-party/json)

#SDK
add_definitions(-DSYMBOLSCANNER_EXPORTS)
add_library(astina_scanner SHARED
	${CMAKE_CURRENT_SOURCE_DIR}/SymbolScanner.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/board.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/utility.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/device_manager.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/streamer.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/sensor.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/frameset.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/frame.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/calibration.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/sensor_info.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/pointcloud.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/preset.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/stream_config.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/multithreader.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/alignment.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/open3d_090.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/corner_visualizer.cpp
)
set_target_properties(astina_scanner PROPERTIES PUBLIC_HEADER SymbolScanner.h)
target_include_directories(astina_scanner PRIVATE ${Open3D_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ../../include ${json_DIR}/include include)
target_link_libraries(astina_scanner spdlog_header_only fmt::fmt ${Open3D_LIBRARIES} ${OpenCV_LIBS} ${DEPENDENCIES} glu32 opengl32 glfw3)
set_target_properties (astina_scanner PROPERTIES FOLDER Examples/astina)
install(TARGETS astina_scanner RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

# set_target_properties(astina_scanner PROPERTIES VERSION ${PROJECT_VERSION})

#endurance test
add_executable(rs-astina endurance_test.cpp)
add_dependencies(rs-astina astina_scanner)
target_include_directories(rs-astina PRIVATE ${Open3D_INCLUDE_DIRS} . ./third-party/cxxopts/include ../../include)
link_directories(${CMAKE_INSTALL_LIBDIR})
target_link_libraries(rs-astina spdlog_header_only fmt::fmt ${Open3D_LIBRARIES} astina_scanner)
set_target_properties (rs-astina PROPERTIES FOLDER Examples/astina)
install(TARGETS rs-astina RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
