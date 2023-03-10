cmake_minimum_required(VERSION 2.8)
project(GF_ORB_SLAM2)

# message("ROS version = " $ENV{ROS_DISTRO})

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

IF(NOT DEFINED ORB_LIB_TYPE)
    SET (ORB_LIB_TYPE "STATIC") # "SHARED"
ENDIF()

IF(NOT DEFINED ENABLE_CUDA_IN_OPENCV)
    SET(ENABLE_CUDA_IN_OPENCV True)
ENDIF()
message("Enable cuda support: " ${ENABLE_CUDA_IN_OPENCV})


# Check architecture
IF(${CMAKE_SYSTEM_PROCESSOR} MATCHES "arm")
    add_definitions("-DENABLE_SSE")
    MESSAGE("SSE flags: " ${SSE_FLAGS})
    # Optimized for Jetson TX2 only; refer to https://devtalk.nvidia.com/default/topic/1028179/jetson-tx2/gcc-options-for-tx2
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}     -Wl,--no-as-needed -pthread -fopenmp -Wall -O3 -ffast-math -flto -march=armv8-a+crypto -mcpu=cortex-a57+crypto")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wl,--no-as-needed -pthread -fopenmp -Wall -O3 -ffast-math -flto -march=armv8-a+crypto -mcpu=cortex-a57+crypto")
ELSE()
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}     -Wl,--no-as-needed -pthread -fopenmp -Wall -O3 -march=native ")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wl,--no-as-needed -pthread -fopenmp -Wall -O3 -march=native")
ENDIF()

# Add CUDA support
if(ENABLE_CUDA_IN_OPENCV)
    find_package(CUDA REQUIRED)
    set(CUDA_PROPAGATE_HOST_FLAGS OFF)
    SET(CUDA_HOST_COMPILER /usr/bin/g++)
    LIST(APPEND CUDA_NVCC_FLAGS "--compiler-options -fno-strict-aliasing -use_fast_math")
    #LIST(APPEND CUDA_NVCC_FLAGS "--compiler-options -Xcompiler -fpermissive")
endif()

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   if(ENABLE_CUDA_IN_OPENCV)
       set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -std=c++11")
       set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -DUSE_NVTX")
       set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_NVTX")
   endif()
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   if(ENABLE_CUDA_IN_OPENCV)
       set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -std=c++0x")
       set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -DUSE_NVTX")
       set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_NVTX")
   endif()
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

IF(SLAM_P_P_SHORT_CXSPARSE)
        ADD_DEFINITIONS(-D__CXSPARSE_SHORT)
ENDIF(SLAM_P_P_SHORT_CXSPARSE)

IF(SLAM_P_P_SHORT_CHOLMOD)
        ADD_DEFINITIONS(-D__CHOLMOD_SHORT)
ENDIF(SLAM_P_P_SHORT_CHOLMOD)

IF(SLAM_P_P_FLAT_SYSTEM_ALIGNED_MEMORY)
        ADD_DEFINITIONS(-D__FLAT_SYSTEM_ALIGNED_MEMORY)
ENDIF(SLAM_P_P_FLAT_SYSTEM_ALIGNED_MEMORY)

# support for GPU BLAS
#IF(SLAM_P_P_GPU_BLAS)
#        ADD_DEFINITIONS(
#                -DGPU_BLAS
#                -D__CHANNEL_DESCRIPTOR_H__
#                -D__CUDA_RUNTIME_H__)
#ENDIF(SLAM_P_P_GPU_BLAS)

ADD_DEFINITIONS(-DCERES_NO_PROTOCOL_BUFFERS)

IF(SLAM_P_P_LINEAR_SOLVER_TYPE)
        ADD_DEFINITIONS("-D__LINEAR_SOLVER_OVERRIDE=${SLAM_P_P_LINEAR_SOLVER_TYPE}")
ENDIF(SLAM_P_P_LINEAR_SOLVER_TYPE)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)



# customize opencv dir
# if ($ENV{ROS_DISTRO} STREQUAL "indigo")
#     set( OpenCV_DIR /opt/opencv2/share/OpenCV )
#     find_package(OpenCV 2.4.13 REQUIRED core ocl)
# elseif ($ENV{ROS_DISTRO} STREQUAL "kinetic")
#     set( OpenCV_DIR /opt/opencv3/share/OpenCV )
#     find_package(OpenCV 3.4.0 REQUIRED)
# endif()
find_package(OpenCV 3.4.1 REQUIRED
                 PATHS /opt/opencv3
                 NO_DEFAULT_PATH)
    
# customize eigen dir
# find_package(Eigen3 3.1.0 REQUIRED)
set( EIGEN3_INCLUDE_DIR /opt/eigen33/include/eigen3/ )
# set( EIGEN3_INCLUDE_DIR /usr/include/eigen3 )
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

# GFlags
find_package(GFlags REQUIRED)
MESSAGE("gflags include dirs: " ${GFLAGS_INCLUDE_DIR})

find_package(Boost  REQUIRED COMPONENTS thread system filesystem)
MESSAGE("Boost include dirs: " ${Boost_INCLUDE_DIRS})



include_directories(
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/Thirdparty/SLAM++/include
    ${PROJECT_SOURCE_DIR}/Thirdparty/sse2neon/
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${Pangolin_INCLUDE_DIRS}
    ${ARMADILLO_INCLUDE_DIR}
    ${Boost_INCLUDE_DIRS}
    ${GTest_INCLUDE_DIR}
    ${GFLAGS_INCLUDE_DIR}
    )

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

if(ENABLE_CUDA_IN_OPENCV)
    cuda_include_directories(
      ${CUDA_TOOLKIT_ROOT_DIR}/samples/common/inc
    )
    cuda_add_library(${PROJECT_NAME} ${ORB_LIB_TYPE}
      include/Util_cuda.hpp
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
      src/FAST_NEON.cc
      src/cuda/Allocator_gpu.cu
      src/cuda/Fast_gpu.cu
      src/cuda/Orb_gpu.cu
      src/cuda/Cuda.cu
      ${PROJECT_SOURCE_DIR}/Thirdparty/SLAM++/src/good_graph_testbed/BAOptimizer.cpp
    )
    IF(${CMAKE_SYSTEM_PROCESSOR} MATCHES "arm")
        cuda_add_library(${PROJECT_NAME} ${ORB_LIB_TYPE} src/FAST_NEON.cc)
    ENDIF()
else()
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
        ${PROJECT_SOURCE_DIR}/Thirdparty/SLAM++/src/good_graph_testbed/BAOptimizer.cpp
    )
    IF(${CMAKE_SYSTEM_PROCESSOR} MATCHES "arm")
        set(SRCS ${SRCS} src/FAST_NEON.cc)
    ENDIF()
    add_library(${PROJECT_NAME} ${ORB_LIB_TYPE} ${SRCS})
endif()

# SLAM ++ header files
FILE(GLOB_RECURSE SLAM_EXAMPLE_HEADERS
    ${PROJECT_SOURCE_DIR}/Thirdparty/SLAM++/include/good_graph_testbed/*.h
    ${PROJECT_SOURCE_DIR}/Thirdparty/SLAM++/include/slam/*.h)

if (${ORB_LIB_TYPE} STREQUAL "STATIC")
    set(LIBS
        ${GFLAGS_LIBRARIES}
        ${OpenCV_LIBS}
        # ${EIGEN3_LIBS}
        ${Pangolin_LIBRARIES}
        ${OpenBLAS_LIBRARIES}
        ${ARMADILLO_LIBRARIES}
        ${Boost_LIBRARIES}
        ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.a
        ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.a
    )
    SET(SLAM_PLUS_PLUS_LIBRARIES
        ${PROJECT_SOURCE_DIR}/Thirdparty/SLAM++/lib/libeigen.a
        ${PROJECT_SOURCE_DIR}/Thirdparty/SLAM++/lib/libcxsparse.a
        ${PROJECT_SOURCE_DIR}/Thirdparty/SLAM++/lib/libcsparse.a
        ${PROJECT_SOURCE_DIR}/Thirdparty/SLAM++/lib/libcholmod.a
        ${PROJECT_SOURCE_DIR}/Thirdparty/SLAM++/lib/libcamd_dlong.a
        ${PROJECT_SOURCE_DIR}/Thirdparty/SLAM++/lib/libslampp_base.a
    )
else()
    set(LIBS
        ${GFLAGS_LIBRARIES}
        ${OpenCV_LIBS}
        # ${EIGEN3_LIBS}
        ${Pangolin_LIBRARIES}
        ${OpenBLAS_LIBRARIES}
        ${ARMADILLO_LIBRARIES}
        ${Boost_LIBRARIES}
        ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
        ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
    )
    SET(SLAM_PLUS_PLUS_LIBRARIES
        ${PROJECT_SOURCE_DIR}/Thirdparty/SLAM++/lib/libeigen.so
        ${PROJECT_SOURCE_DIR}/Thirdparty/SLAM++/lib/libcxsparse.so
        ${PROJECT_SOURCE_DIR}/Thirdparty/SLAM++/lib/libcsparse.so
        ${PROJECT_SOURCE_DIR}/Thirdparty/SLAM++/lib/libcholmod.so
        ${PROJECT_SOURCE_DIR}/Thirdparty/SLAM++/lib/libcamd_dlong.so
        ${PROJECT_SOURCE_DIR}/Thirdparty/SLAM++/lib/libslampp_base.so
    )
endif()

if(ENABLE_CUDA_IN_OPENCV)
    target_link_libraries(${PROJECT_NAME} ${LIBS} nvToolsExt ${SLAM_PLUS_PLUS_LIBRARIES})
else()
    target_link_libraries(${PROJECT_NAME} ${LIBS} ${SLAM_PLUS_PLUS_LIBRARIES})
endif()


# Build examples

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/tools)
add_executable(bin_vocabulary tools/bin_vocabulary.cc)
target_link_libraries(bin_vocabulary ${PROJECT_NAME})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular)

add_executable(mono_general Examples/Monocular/mono_general.cc ${SLAM_EXAMPLE_HEADERS})
target_link_libraries(mono_general
    GL
    glut
    GLU
    ${PROJECT_NAME}
    ${Boost_LIBRARIES}
    ${SLAM_PLUS_PLUS_LIBRARIES}
    ${OpenBLAS_LIBRARIES}
    ${ARMADILLO_LIBRARIES}
    )

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Stereo)

add_executable(stereo_general Examples/Stereo/stereo_general.cc ${SLAM_EXAMPLE_HEADERS})
target_link_libraries(stereo_general
    GL
    glut
    GLU
    ${PROJECT_NAME}
    ${Boost_LIBRARIES}
    ${SLAM_PLUS_PLUS_LIBRARIES}
    ${OpenBLAS_LIBRARIES}
    ${ARMADILLO_LIBRARIES}
    )


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
#    ./test/test_MapIO.cpp
    ./test/main.cpp
    )
target_link_libraries(testObs
    GL
    glut
    GLU
    ${PROJECT_NAME}
    ${GTest_LIBRARIES}
    ${SLAM_PLUS_PLUS_LIBRARIES}
    ${OpenBLAS_LIBRARIES}
    ${ARMADILLO_LIBRARIES}
    )
