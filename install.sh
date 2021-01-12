cd lepton
mkdir build
cd build
cmake ..
sudo make install
cd ../..

cd libbsn
mkdir build 
cd build 
cmake .. 
sudo make install
cd ../..

cd ~/catkin_ws/ 
catkin_make