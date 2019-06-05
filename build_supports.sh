echo "Configuring and building Thirdparty/DBoW2 ..."

cd Thirdparty/DBoW2
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release # -DCUDA_USE_STATIC_CUDA_RUNTIME=OFF
make -j

cd ../../g2o

echo "Configuring and building Thirdparty/g2o ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j

cd ../../../

echo "Uncompress vocabulary ..."

cd ../ORB_Data
tar -xf ORBvoc.txt.tar.gz

cd ../gf_orb_slam2
echo "Converting vocabulary to binary"
./tools/bin_vocabulary
