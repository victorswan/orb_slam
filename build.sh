dir=`pwd`
echo $dir

echo "Configuring and building GF-ORB_SLAM2 ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DENABLE_CUDA_IN_OPENCV=False -DORB_LIB_TYPE:STRING="STATIC" # "SHARED" # 
make -j4
#sudo make install

# cd ..
# echo "Converting vocabulary to binary"
# ./tools/bin_vocabulary

cd $dir
