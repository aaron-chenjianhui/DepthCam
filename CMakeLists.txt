cmake_minimum_required(VERSION 2.8)

set(CMAKE_CXX_STANDARD 11)

set(CMAKE_BUILD_TYPE Debug)
#set(CMAKE_BUILD_TYPE Release)

set(INCLUDE_PATH include)
set(TARGET_LIB tycam)


#set(SRC_LISTS src/depthcam.cpp)
set(EXE_LISTS
#multidev
#depthcam
#CamDriver
test_pcd)
#set(SRC_LISTS
#        src/multidev.cpp
#        src/depthcam.cpp)


# ========================================
# === common, pcl is too slow, so build a
# === lib to speed up
# ========================================
set(COMMON_SOURCES
    common/MatViewer.cpp
    common/PointCloudViewer.cpp
    )

add_library(sample_common STATIC ${COMMON_SOURCES})


# ========================================
# === OpenCV
# ========================================
find_package(OpenCV REQUIRED)
if (NOT OpenCV_FOUND)
    message(FATAL_ERROR "OpenCV library not found")
else()
    include_directories(${OpenCV_INCLUDE_DIRS})
    include_directories(${OpenCV2_INCLUDE_DIRS})
    # if(CMAKE_BUILD_TYPE STREQUAL Debug)
    # link_directories(${OpenCV_LIB_DIR_DBG})
    # else()
    # link_directories(${OpenCV_LIB_DIR_OPT})
    # endif()
    link_directories(${OpenCV_LIB_DIR})
endif()


# ========================================
# === PCL
# ========================================
find_package(PCL)
if (NOT PCL_FOUND)
    message(WARNING "PCL library not found, some samples will not be compiled")
else()
    add_definitions(-DHAVE_PCL)
    include_directories(${PCL_INCLUDE_DIRS})
    link_directories(${PCL_LIBRARY_DIRS})
    # target_link_libraries(MY_TARGET_NAME ${PCL_LIBRARIES})
endif()


# ========================================
# === Eigen
# ========================================
#find_package(Eigen3)
#if (NOT EIGEN3_FOUND)
#  message(FATAL_ERROR "Eigen library not found")
#else()
#  include_directories(${EIGEN3_INCLUDE_DIR})
#endif()

include_directories(${INCLUDE_PATH})

foreach (exe_name ${EXE_LISTS})
    get_filename_component(spath "src/${exe_name}" ABSOLUTE)
        message("spath is ${spath}.cpp")

    if (EXISTS "${spath}.cpp")
#        file(GLOB source "${exe_name}.cpp")
#            message("source is ${exe_name}.cpp")
        add_executable(${exe_name} src/${exe_name}.cpp)
        add_dependencies(${exe_name} sample_common ${TARGET_LIB})
        target_link_libraries(${exe_name} sample_common ${TARGET_LIB} ${OpenCV_LIBS} ${PCL_LIBRARIES})

        set_target_properties(${exe_name} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/)

    endif ()

endforeach ()
