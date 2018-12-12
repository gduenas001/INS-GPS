cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(feature_extractor)

find_package(PCL 1.8 REQUIRED)


file(GLOB SOURCES ${PROJECT_SOURCE_DIR}/src/*.cpp)
file(GLOB H_FILES ${PROJECT_SOURCE_DIR}/src/h_files/*.h)


SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread")


include_directories(${PCL_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/src/h_files)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable (FE	${SOURCES} ${H_FILES})
target_link_libraries (FE ${PCL_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})

