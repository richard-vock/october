cmake_minimum_required(VERSION 2.8.8)
project(october)

set (october_VERSION_MAJOR 0)
set (october_VERSION_MINOR 1)

include_directories("${PROJECT_SOURCE_DIR}")
include_directories("${PROJECT_BINARY_DIR}")
include_directories("${PROJECT_SOURCE_DIR}/include")
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/modules)

if(CMAKE_COMPILER_IS_GNUCXX)
    if (NOT WIN32)
        add_definitions(-fPIC)
    endif()
	add_definitions(-O3)
	add_definitions(-g)
	add_definitions(-std=c++14)
    add_definitions(-Wall)
    add_definitions(-Wno-unused-function)
    add_definitions(-Wno-unused-local-typedefs)
    add_definitions(-Wno-maybe-uninitialized)
    #add_definitions(-U__STRICT_ANSI__)
    add_definitions(-DM_LOG2E=1.44269504088896340736)
endif()

find_package(Eigen)
find_package(OpenCV COMPONENTS core imgproc highgui)

file (GLOB_RECURSE obj RELATIVE "${PROJECT_SOURCE_DIR}" "src/*.cpp")
if (EIGEN_FOUND AND OPENCV_CORE_FOUND AND OPENCV_IMGPROC_FOUND AND OPENCV_HIGHGUI_FOUND)
	include_directories(${EIGEN_INCLUDE_DIRS})
	include_directories(${OpenCV_INCLUDE_DIRS})

    add_library(october SHARED ${obj})
    target_link_libraries(october ${OpenCV_LIBRARIES} "dl")

	# install binary
    if (WIN32)
        install (FILES "${PROJECT_BINARY_DIR}/liboctober.dll.a" DESTINATION lib)
        #install (FILES "${PROJECT_BINARY_DIR}/liboctober.dll" DESTINATION lib)
        install (FILES "${PROJECT_BINARY_DIR}/liboctober.dll" DESTINATION bin)
    else()
        install (TARGETS october DESTINATION lib)
    endif()

	# install header
    install (DIRECTORY include/ DESTINATION include/october)
endif()
