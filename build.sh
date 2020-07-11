cd /
apt install -y libproj-dev
git clone https://github.com/BOpermanis/SP-SLAM.git
wget https://github.com/PointCloudLibrary/pcl/archive/pcl-1.8.0.tar.gz
tar -xf pcl-1.8.0.tar.gz
cd pcl-pcl-1.8.0 && mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j2 install
