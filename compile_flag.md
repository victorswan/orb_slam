 -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-O3 -DNDEBUG -march=native" -DCMAKE_C_FLAGS_RELEASE="-O3 -DNDEBUG -march=native"


# compile command
catkin build -j2 -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-O3 -DNDEBUG -march=native" -DCMAKE_C_FLAGS_RELEASE="-O3 -DNDEBUG -march=native" -DENABLE_CUDA_IN_OPENCV=False