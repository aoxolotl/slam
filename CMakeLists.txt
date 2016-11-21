cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project("Come On And SLAM!")

# Set cmake prefix path to enable cmake to find freenect2
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} $ENV{HOME}/freenect2/lib/cmake/freenect2)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR})
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} ${CMAKE_CURRENT_SOURCE_DIR})

find_package(freenect2 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL 1.3 REQUIRED COMPONENTS common io filters surface segmentation visualization)

include_directories("/usr/include/libusb-1.0/")
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${freenect2_INCLUDE_DIR})
include_directories("${PROJECT_SOURCE_DIR}/include")

link_directories(${PCL_LIBRARY_DIRS})
link_directories("${PROJECT_SOURCE_DIR}/libs")

add_definitions(${PCL_DEFINITIONS})

#add sources
file(GLOB_RECURSE SOURCES "modules/*.cpp")
file(GLOB_RECURSE HEADERS "modules/*.h")
file(GLOB_RECURSE HEADERS "modules/*.hpp")

MESSAGE("SOURCES: ${SOURCES}")

add_executable(main ${SOURCES} ${HEADERS})

target_link_libraries(main libdxl.a libapriltag.a m ${freenect2_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_FILTERS_LIBRARIES} ${PCL_SURFACE_LIBRARIES} ${PCL_SEGMENTATION_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES} ${OpenCV_LIBS})
