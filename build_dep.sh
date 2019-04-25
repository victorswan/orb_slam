# assuming ros-indigo has been configured properly, and the gcc is the standard 4.8.5 for ubuntu 14.04.5 LTS
# also, make sure no gfortran is installed; otherwise openblas will build lapack and blas, which are leading to worse performance of subset selection than lapack-dev
# sudo apt-get remove libgfortran-4.*-dev

# make sure no openblas being installed
sudo apt-get remove libopenblas-base

# build openblas with single-thread
cd /mnt/DATA/SDK/OpenBLAS-0.2.20
make USE_THREAD=0 
sudo make PREFIX=/opt/OpenBLAS install

# build armadillo
cd /mnt/DATA/SDK/armadillo-8.400.0
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX:PATH="/opt/armadillo" -DCMAKE_INSTALL_LIBDIR:PATH="lib" -DCMAKE_BUILD_TYPE:STRING="Release" -Dopenblas_LIBRARY:FILEPATH="/opt/OpenBLAS/lib/libopenblas.so" -Dopenblasp_LIBRARY:FILEPATH="/opt/OpenBLAS/lib/libopenblas.so" -Dopenblaso_LIBRARY:FILEPATH="/opt/OpenBLAS/lib/libopenblas.so" -DLAPACK_LIBRARY:FILEPATH="/opt/OpenBLAS/lib/libopenblas.so" 
make -j
sudo make install

# # build opencv2
# cd /mnt/DATA/SDK/opencv-2.4.13.6
# mkdir build
# cd build
# cmake .. -DCMAKE_INSTALL_PREFIX:PATH="/opt/opencv3" -DBUILD_TBB:BOOL="1" -DWITH_CUFFT:BOOL="0" -DWITH_TBB:BOOL="1" -DWITH_CUDA:BOOL="0" -DCUDA_HOST_COMPILATION_CPP:BOOL="0" -DCMAKE_BUILD_TYPE:STRING="Release" -DWITH_OPENMP:BOOL="1" -DCUDA_PROPAGATE_HOST_FLAGS:BOOL="0" -DCUDA_64_BIT_DEVICE_CODE:BOOL="0" -DCUDA_ATTACH_VS_BUILD_RULE_TO_CUDA_FILE:BOOL="0" -DBUILD_opencv_gpu:BOOL="0" 
# make -j
# sudo make install

# build opencv 3.4
cd /mnt/DATA/SDK/opencv-3.4.1
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX:PATH="/opt/opencv3" -DBUILD_TBB:BOOL="1" -DWITH_TBB:BOOL="1" -DCMAKE_BUILD_TYPE:STRING="Release" -DWITH_OPENMP:BOOL="1"  -DBUILD_opencv_gpu:BOOL="0" -DOPENCV_EXTRA_MODULES_PATH:PATH="/mnt/DATA/SDK/opencv_contrib-3.4.1/modules" -DBUILD_opencv_cudaobjdetect:BOOL="0" -DWITH_CUFFT:BOOL="0" -DBUILD_opencv_cudaimgproc:BOOL="0" -DBUILD_opencv_cudastereo:BOOL="0" -DBUILD_opencv_cudaoptflow:BOOL="0" -DBUILD_opencv_cudabgsegm:BOOL="0" -DBUILD_opencv_cudaarithm:BOOL="0" -DWITH_CUDA:BOOL="0" -DOPENCV_ENABLE_NONFREE:BOOL="1" -DBUILD_opencv_cudacodec:BOOL="0" -DWITH_CUBLAS:BOOL="0" -DBUILD_opencv_cudawarping:BOOL="0" -DBUILD_opencv_cudafilters:BOOL="0" -DCUDA_64_BIT_DEVICE_CODE:BOOL="0" -DBUILD_opencv_cudafeatures2d:BOOL="0" -DBUILD_opencv_cudalegacy:BOOL="0" 
make -j
sudo make install

# build Pangolin
sudo apt-get install libglew-dev

cd /mnt/DATA/SDK/Pangolin
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX:PATH="/opt/Pangolin" -DCMAKE_BUILD_TYPE:STRING="Release"
make -j
sudo make install
