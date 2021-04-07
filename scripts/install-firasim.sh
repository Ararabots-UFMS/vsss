sudo apt-get install git build-essential cmake qt5-default libqt5opengl5-dev libgl1-mesa-dev libglu1-mesa-dev libprotobuf-dev protobuf-compiler libode-dev libboost-dev
cd /tmp
git clone https://github.com/jpfeltracco/vartypes.git
cd vartypes
mkdir build
cd build
cmake ..
make
sudo make install

mkdir $ROS_ARARA_ROOT"firasim_ws" 
cd $ROS_ARARA_ROOT"firasim_ws"
git clone https://github.com/robocin/FIRASim.git
cd FIRASim
mkdir build
cd build
cmake ..
make