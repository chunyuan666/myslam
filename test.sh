clear
cd build
cmake ..
make -j4
cd ..
./bin/stereo_kitti ./config/KITTI00-02.yaml /home/yuan/slam_data/data_odometry_gray/dataset/sequences/00