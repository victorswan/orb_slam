echo "Building ROS nodes"

cd Examples/ROS/GF_ORB_SLAM2
mkdir build
cd build
cmake .. -DROS_BUILD_TYPE=Release # -DCUDA_USE_STATIC_CUDA_RUNTIME=OFF
make -j
