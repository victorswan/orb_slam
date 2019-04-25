cmake_minimum_required(VERSION 2.8)
project(GF_ORB_SLAM2)

message("ROS version = " $ENV{ROS_DISTRO})

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wl,--no-as-needed -pthread  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wl,--no-as-needed -pthread  -Wall   -O3 -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

# customize opencv dir
 if ($ENV{ROS_DISTRO} STREQUAL "indigo")
     set( OpenCV_DIR /opt/opencv2/share/OpenCV )
     find_package(OpenCV 2.4.13 REQUIRED core ocl)
 elseif ($ENV{ROS_DISTRO} STREQUAL "kinetic")
     # set( OpenCV_DIR /opt/opencv-3.4.1/share/OpenCV )
     set( OpenCV_DIR /opt/opencv3/share/OpenCV )
     find_package(OpenCV 3.4.0 REQUIRED)
 endif()
#set( OpenCV_DIR /opt/opencv3/share/OpenCV )
#find_package(OpenCV 3.4.0 REQUIRED)
    
# customize eigen dir
# find_package(Eigen3 3.1.0 REQUIRED)
set( EIGEN3_INCLUDE_DIR /usr/include/eigen3 )
MESSAGE("EIGEN include dir: " ${EIGEN3_INCLUDE_DIR})
# MESSAGE("EIGEN libs: " ${EIGEN3_LIBS})

# customize pangolin dir
# set(Pangolin_DIR /opt/Pangolin/lib/cmake/Pangolin/)
# find_package(Pangolin REQUIRED)
set(Pangolin_INCLUDE_DIRS /opt/Pangolin/include/)
set(Pangolin_LIBRARIES /opt/Pangolin/lib/libpangolin.so)
MESSAGE("Pangolin include dir: " ${Pangolin_INCLUDE_DIRS})


# customize openblas dir
set(OpenBLAS_LIBRARIES /opt/OpenBLAS/lib/libopenblas.so)
MESSAGE("OpenBLAS libs:" ${OpenBLAS_LIBRARIES})

# customize armadillo dir
set(ARMADILLO_INCLUDE_DIR /opt/armadillo/include)
set(ARMADILLO_LIBRARIES /opt/armadillo/lib/libarmadillo.so)
MESSAGE("ARMADILLO libs: " ${ARMADILLO_LIBRARIES})

# customize gtest dir
set(GTest_INCLUDE_DIR /usr/src/gtest/src)
set(GTest_LIBRARIES /usr/src/gtest/libgtest.a)
MESSAGE("gtest libs: " ${GTest_LIBRARIES})

find_package(Boost    REQUIRED COMPONENTS thread system filesystem)

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${OpenCV_INCLUDE_DIRS}
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
${ARMADILLO_INCLUDE_DIR}
${Boost_INCLUDE_DIRS}
${GTest_INCLUDE_DIR}
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

set(SRCS
    include/Util.hpp
    src/System.cc
    src/Observability.cc
    src/Tracking.cc
    src/LocalMapping.cc
    src/LoopClosing.cc
    src/ORBextractor.cc
    src/ORBmatcher.cc
    src/FrameDrawer.cc
    src/Converter.cc
    src/MapPoint.cc
    src/KeyFrame.cc
    src/Map.cc
    src/MapDrawer.cc
    src/Optimizer.cc
    src/PnPsolver.cc
    src/Frame.cc
    src/KeyFrameDatabase.cc
    src/Sim3Solver.cc
    src/Initializer.cc
    src/Viewer.cc
    )

add_library(${PROJECT_NAME} SHARED
${SRCS}
)

set(LIBS
${OpenCV_LIBS}
# ${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${OpenBLAS_LIBRARIES}
${ARMADILLO_LIBRARIES}
${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
)

target_link_libraries(${PROJECT_NAME}
${LIBS}
)

# Build examples

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/tools)
add_executable(bin_vocabulary
tools/bin_vocabulary.cc)
target_link_libraries(bin_vocabulary ${PROJECT_NAME})


set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Stereo)

add_executable(stereo_general
Examples/Stereo/stereo_general.cc)
target_link_libraries(stereo_general
    GL
    glut
    GLU
    ${PROJECT_NAME}
    ${Boost_LIBRARIES}
    ${OpenBLAS_LIBRARIES}
    ${ARMADILLO_LIBRARIES})

# Build test cases

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/test)

add_executable(testObs
    ${SRCS}
    ./test/test_Kine_1.cpp
    ./test/test_Kine_2.cpp
    ./test/test_Jacobian.cpp
    ./test/test_Greedy.cpp
    ./test/test_GoodMap.cpp
    ./test/test_Stereo.cpp
    ./test/main.cpp
    )
target_link_libraries(testObs
    GL  
    glut
    GLU
    ${PROJECT_NAME}
    ${OpenBLAS_LIBRARIES}
    ${ARMADILLO_LIBRARIES}
    ${GTest_LIBRARIES}
)
