#!/bin/bash
export ROS_DISTRO=jazzy

locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings

sudo apt install software-properties-common curl -y
sudo add-apt-repository universe
sudo add-apt-repository ppa:deadsnakes/ppa

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y \
    ros-dev-tools \
    ros-$ROS_DISTRO-desktop \
    python3-pip \
    ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
    ros-dev-tools \
    python3-colcon-common-extensions \
    ros-$ROS_DISTRO-diagnostic-updater \
    libgpiod-dev \
    gpiod \
    ros-$ROS_DISTRO-xacro \
    python3-pyaudio \
    alsa-utils \
    libportaudio2 \
    screen \
    micro \
    mc \
    nano \
    python-is-python3 \
    python3.10 \
    python3.10-venv \
    python3.10-distutils \
    python3.10-dev \
    libi2c-dev 

sudo apt-get update 
sudo apt-get upgrade -y

curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh

cd /psyche

python3.10 -m venv .venv
source .venv/bin/activate

pip install -r requirements.txt
sudo rosdep init
rosdep update 
    
cd /psyche/src
git clone https://github.com/autonomylab/create_robot.git
git clone https://github.com/hiwad-aziz/ros2_mpu6050_driver.git
git clone https://github.com/revyos-ros/libcreate.git
cd libcreate
git checkout fix-std-string
cd ../ros2_mpu6050_driver
patch -p1 < /psyche/mpu.patch
cd /psyche

/bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y"
/bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && \
    colcon build"

BASHRC=$HOME/.bashrc
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> $BASHRC
echo "export ROS_DOMAIN_ID=42" >> $BASHRC
echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> $BASHRC
echo "alias ros_source=\"source /opt/ros/$ROS_DISTRO/setup.bash\"" >> $BASHRC
echo "source /psyche/install/setup.bash" >> $BASHRC
echo "alias psyche_source=\"source /psyche/install/setup.bash\"" >> $BASHRC
echo "alias venv=\"source /psyche/.venv/bin/activate\"" >> $BASHRC
echo "alias resource=\"ros_source && psyche_source\"" >> $BASHRC
echo "alias psyche=\"resource && venv && ros2\"" >> $BASHRC
echo "alias pbuild=\"here=$(pwd) && cd /psyche && venv && colcon build && resource && venv && cd '\$here' && deactivate\"" >> $BASHRC
