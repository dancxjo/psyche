#!/bin/bash
export ROS_DISTRO=rolling

sudo apt install software-properties-common curl -y
sudo add-apt-repository universe
sudo add-apt-repository ppa:deadsnakes/ppa
sudo apt update
sudo apt install -y \
    ros-dev-tools \
    ros-rolling-desktop \
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
    python3.10-dev

sudo apt-get update 
sudo apt-get upgrade -y

cd /psyche

python3.10 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
sudo rosdep init
rosdep update 

pip install -qU \
    langchain \
    langchain-community \
    langchain-openai \
    sentence_splitter \
    langchain-text-splitters \
    faiss-cpu \
    langchainhub \
    langchain_experimental \
    gpiod \
    SpeechRecognition \
    openai-whisper \
    aiohttp

    
cd /psyche/src
git clone https://github.com/autonomylab/create_robot.git
git clone https://github.com/hiwad-aziz/ros2_mpu6050_driver.git
git clone https://github.com/revyos-ros/libcreate.git
cd libcreate
git checkout fix-std-string
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
echo "alias resource=\"ros_source && psyche_source\"" >> $BASHRC
echo "alias psyche=\"resource && ros2\"" >> $BASHRC
echo "alias pbuild=\"colcon build --symlink-install\"" >> $BASHRC
