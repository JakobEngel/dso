SET(PROJECT_NAME DSO)

PROJECT(${PROJECT_NAME})
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
#set(CMAKE_VERBOSE_MAKEFILE ON)


set(BUILD_TYPE Release)
#set(BUILD_TYPE RelWithDebInfo)

set(EXECUTABLE_OUTPUT_PATH bin)
set(LIBRARY_OUTPUT_PATH lib)
set(INCLUDE_OUTPUT_PATH include/dso)
set(CMAKE_OUTPUT_PATH ${LIBRARY_OUTPUT_PATH}/cmake/dso)
set(PKGCONFIG_OUTPUT_PATH ${LIBRARY_OUTPUT_PATH}/pkgconfig)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# required libraries
find_package(SuiteParse REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS system thread) 

# optional libraries
find_package(LibZip QUIET)
find_package(Pangolin 0.2 QUIET)
find_package(OpenCV QUIET)

# flags
add_definitions("-DENABLE_SSE")
set(CMAKE_CXX_FLAGS
   "${SSE_FLAGS} -O3 -g -std=c++0x -march=native"
#   "${SSE_FLAGS} -O3 -g -std=c++0x -fno-omit-frame-pointer"
)
option(DSO_BUILD_STATIC_LIBRARY        "Build a static dso library, instead of shared" ON)

if (MSVC)
     set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /EHsc")
endif (MSVC)

if(NOT WIN32)
    option(DSO_BUILD_PKGCONFIG "Build pkg-config .pc file for DSO" ON)
endif(NOT WIN32)

# Sources files
set(dso_SOURCE_FILES
  ${PROJECT_SOURCE_DIR}/src/FullSystem/FullSystem.cpp
  ${PROJECT_SOURCE_DIR}/src/FullSystem/FullSystemOptimize.cpp
  ${PROJECT_SOURCE_DIR}/src/FullSystem/FullSystemOptPoint.cpp
  ${PROJECT_SOURCE_DIR}/src/FullSystem/FullSystemDebugStuff.cpp
  ${PROJECT_SOURCE_DIR}/src/FullSystem/FullSystemMarginalize.cpp
  ${PROJECT_SOURCE_DIR}/src/FullSystem/Residuals.cpp
  ${PROJECT_SOURCE_DIR}/src/FullSystem/CoarseTracker.cpp
  ${PROJECT_SOURCE_DIR}/src/FullSystem/CoarseInitializer.cpp
  ${PROJECT_SOURCE_DIR}/src/FullSystem/ImmaturePoint.cpp
  ${PROJECT_SOURCE_DIR}/src/FullSystem/HessianBlocks.cpp
  ${PROJECT_SOURCE_DIR}/src/FullSystem/PixelSelector2.cpp
  ${PROJECT_SOURCE_DIR}/src/OptimizationBackend/EnergyFunctional.cpp
  ${PROJECT_SOURCE_DIR}/src/OptimizationBackend/AccumulatedTopHessian.cpp
  ${PROJECT_SOURCE_DIR}/src/OptimizationBackend/AccumulatedSCHessian.cpp
  ${PROJECT_SOURCE_DIR}/src/OptimizationBackend/EnergyFunctionalStructs.cpp
  ${PROJECT_SOURCE_DIR}/src/util/settings.cpp
  ${PROJECT_SOURCE_DIR}/src/util/Undistort.cpp
  ${PROJECT_SOURCE_DIR}/src/util/globalCalib.cpp
)

set(DSO_BOOST_LIBRARIES ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY})

include_directories(
  ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_SOURCE_DIR}/thirdparty/Sophus
  ${PROJECT_SOURCE_DIR}/thirdparty/sse2neon
  ${EIGEN3_INCLUDE_DIR}
) 


# decide if we have pangolin
if (Pangolin_FOUND)
	message("--- found PANGOLIN, compiling dso_pangolin library.")
	include_directories( ${Pangolin_INCLUDE_DIRS} ) 
	set(dso_pangolin_SOURCE_FILES 
	  ${PROJECT_SOURCE_DIR}/src/IOWrapper/Pangolin/KeyFrameDisplay.cpp
	  ${PROJECT_SOURCE_DIR}/src/IOWrapper/Pangolin/PangolinDSOViewer.cpp)
	set(HAS_PANGOLIN 1)
else ()
	message("--- could not find PANGOLIN, not compiling dso_pangolin library.")
	message("    this means there will be no 3D display / GUI available for dso_dataset.")
	set(dso_pangolin_SOURCE_FILES )
	set(HAS_PANGOLIN 0)
endif ()

# decide if we have openCV
if (OpenCV_FOUND)
	message("--- found OpenCV, compiling dso_opencv library.")
	include_directories( ${OpenCV_INCLUDE_DIRS} )
	set(dso_opencv_SOURCE_FILES 
	  ${PROJECT_SOURCE_DIR}/src/IOWrapper/OpenCV/ImageDisplay_OpenCV.cpp
	  ${PROJECT_SOURCE_DIR}/src/IOWrapper/OpenCV/ImageRW_OpenCV.cpp)
	set(HAS_OPENCV 1)
else ()
	message("--- could not find OpenCV, not compiling dso_opencv library.")
	message("    this means there will be no image display, and image read / load functionality.")
	set(dso_opencv_SOURCE_FILES 
	  ${PROJECT_SOURCE_DIR}/src/IOWrapper/ImageDisplay_dummy.cpp
	  ${PROJECT_SOURCE_DIR}/src/IOWrapper/ImageRW_dummy.cpp)
	set(HAS_OPENCV 0)
endif ()

# decide if we have ziplib.
if (LIBZIP_LIBRARY)
	message("--- found ziplib (${LIBZIP_VERSION}), compiling with zip capability.")
	add_definitions(-DHAS_ZIPLIB=1)
	include_directories( ${LIBZIP_INCLUDE_DIR_ZIP} ${LIBZIP_INCLUDE_DIR_ZIPCONF} ) 
else()
	message("--- not found ziplib (${LIBZIP_LIBRARY}), compiling without zip capability.")
	set(LIBZIP_LIBRARY "")
endif()


# compile main library.
include_directories( ${CSPARSE_INCLUDE_DIR} ${CHOLMOD_INCLUDE_DIR})

if (DSO_BUILD_STATIC_LIBRARY)
    message("--- dso static library selected.")
    add_library(dso ${dso_SOURCE_FILES} ${dso_opencv_SOURCE_FILES} ${dso_pangolin_SOURCE_FILES})
    install (TARGETS dso ARCHIVE DESTINATION ${LIBRARY_OUTPUT_PATH} COMPONENT libraries)
else()
    message("--- dso dynamic library selected.")
    add_library(dso SHARED ${dso_SOURCE_FILES} ${dso_opencv_SOURCE_FILES} ${dso_pangolin_SOURCE_FILES})
    target_link_libraries(dso boost_system boost_thread cxsparse)
    if (HAS_ZIPLIB)
        target_link_libraries(dso ${LIBZIP_LIBRARY})
    endif()
    if (HAS_PANGOLIN)
        target_link_libraries(dso ${Pangolin_LIBRARIES})
    endif()
    if (HAS_OPENCV)
        target_link_libraries(dso ${OpenCV_LIBS})
    endif()
    install (TARGETS dso LIBRARY DESTINATION ${LIBRARY_OUTPUT_PATH} COMPONENT libraries)
endif()
#set_property( TARGET dso APPEND_STRING PROPERTY COMPILE_FLAGS -Wall )


if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin") # OSX
    set(BOOST_THREAD_LIBRARY boost_thread-mt)
else()
    set(BOOST_THREAD_LIBRARY boost_thread)
endif()

# build main executable (only if we have both OpenCV and Pangolin)
if (OpenCV_FOUND AND Pangolin_FOUND)
	message("--- compiling dso_dataset.")
	add_executable(dso_dataset ${PROJECT_SOURCE_DIR}/src/main_dso_pangolin.cpp )
    target_link_libraries(dso_dataset dso boost_system cxsparse ${BOOST_THREAD_LIBRARY} ${LIBZIP_LIBRARY} ${Pangolin_LIBRARIES} ${OpenCV_LIBS})
else()
	message("--- not building dso_dataset, since either don't have openCV or Pangolin.")
endif()

file(GLOB dso_HEADER_FILES "${PROJECT_SOURCE_DIR}/src/FullSystem/*.h")

install (FILES ${dso_HEADER_FILES} DESTINATION ${INCLUDE_OUTPUT_PATH}/FullSystem)

file(GLOB dso_HEADER_FILES "${PROJECT_SOURCE_DIR}/src/OptimizationBackend/*.h")

install (FILES ${dso_HEADER_FILES} DESTINATION ${INCLUDE_OUTPUT_PATH}/OptimizationBackend)

file(GLOB dso_HEADER_FILES "${PROJECT_SOURCE_DIR}/src/util/*.h")

install (FILES ${dso_HEADER_FILES} DESTINATION ${INCLUDE_OUTPUT_PATH}/util)

macro(libraries_for_pkgconfig VARNAME)
    foreach(__lib ${ARGN})
        string(STRIP __lib ${__lib})
        string(SUBSTRING ${__lib} 0 1 __lib_is_absolute)
        if (__lib_is_absolute STREQUAL "/")
            get_filename_component(__lib_path ${__lib} PATH)
            get_filename_component(__lib_name ${__lib} NAME_WE)
            string(REGEX REPLACE "^lib" "" __lib_name "${__lib_name}")
            set(${VARNAME} "${${VARNAME}} -L${__lib_path} -l${__lib_name}")
        else()
            set(${VARNAME} "${${VARNAME}} ${__lib}")
        endif()
    endforeach()
endmacro()

# Generate pkgconfig pc file
if(DSO_BUILD_PKGCONFIG)
    if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/dso.pc.in)
        message("-- Found: ${CMAKE_CURRENT_SOURCE_DIR}/dso.pc.in")

        # Set the target name
        set(TARGET_NAME dso)

        # Dependency for DSO
        libraries_for_pkgconfig(${TARGET_NAME}_PKGCONFIG_LIBS
            ${DSO_BOOST_LIBRARIES} ${LIBZIP_LIBRARY} ${Pangolin_LIBRARIES}
            ${OpenCV_LIBS})

        set(${TARGET_NAME}_PKGCONFIG_CFLAGS "${${TARGET_NAME}_PKGCONFIG_CFLAGS} ${BOOST_CFLAGS_OTHER}")

        set(PKGCONFIG_REQUIRES eigen3)
        set(PKGCONFIG_CFLAGS ${${TARGET_NAME}_PKGCONFIG_CFLAGS})
        set(PKGCONFIG_LIBS ${${TARGET_NAME}_PKGCONFIG_LIBS})

        configure_file(dso.pc.in dso.pc @ONLY)
        install(FILES ${CMAKE_CURRENT_BINARY_DIR}/dso.pc DESTINATION ${PKGCONFIG_OUTPUT_PATH})
    else()
        message("-- pkg-config: ${CMAKE_CURRENT_SOURCE_DIR}/dso.pc.in is not available for configuration")
    endif()

endif(DSO_BUILD_PKGCONFIG)

