ARG ROS_DISTRO=iron
ARG ROS_REPO=arm64v8/ros:${ROS_DISTRO}
FROM ${ROS_REPO}

RUN apt update && apt install -y \
    ros-dev-tools \
    python3-pip \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    python3-colcon-common-extensions \
    ros-${ROS_DISTRO}-diagnostic-updater \
    libgpiod-dev \
    gpiod \
    ros-${ROS_DISTRO}-xacro \
    python3-pyaudio \
    alsa-utils \
    libportaudio2 \
    screen \
    micro \
    mc \
    nano \
    python-is-python3 \
    python3-venv \
    python3-distutils \
    python3-dev \
    libi2c-dev \
    ros-${ROS_DISTRO}-usb-cam \
    python3-distutils \
    ros-${ROS_DISTRO}-perception-pcl \
    ros-${ROS_DISTRO}-perception

RUN pip install langchain
RUN pip install langchain-community
RUN pip install langchain-openai
RUN pip install sentence_splitter
RUN pip install langchain-text-splitters
RUN pip install faiss-cpu
RUN pip install langchainhub
RUN pip install langchain_experimental
RUN pip install gpiod
RUN pip install SpeechRecognition
RUN pip install openai-whisper
RUN pip install gTTS
RUN pip install flask
RUN pip install pydub

RUN apt install -y libasound2-dev
RUN pip install soundfile

# RUN apt install -y python3-sphinx build-essential swig libpulse-dev
# RUN apt install -y cython3
# RUN pip install cython
# RUN pip install pocketsphinx

RUN apt install -y ros-$ROS_DISTRO-camera-calibration-parsers

RUN mkdir -p /psyche/src
WORKDIR /psyche/src
RUN git clone https://github.com/autonomylab/create_robot.git
RUN git clone https://github.com/hiwad-aziz/ros2_mpu6050_driver.git
# Don't forget to patch the mpu
RUN git clone --branch fix-std-string https://github.com/revyos-ros/libcreate.git
RUN git clone https://github.com/clydemcqueen/opencv_cam.git
RUN git clone https://github.com/ptrmu/ros2_shared.git

RUN echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> $HOME/.bashrc
RUN echo "export ROS_DOMAIN_ID=42" >> $HOME/.bashrc
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> $HOME/.bashrc
RUN echo "source /psyche/install/setup.bash" >> $HOME/.bashrc
RUN echo "alias ros_source=\"source /opt/ros/$ROS_DISTRO/setup.bash\"" >> $HOME/.bashrc
RUN echo "alias psyche_source=\"source /psyche/install/setup.bash\"" >> $HOME/.bashrc
RUN echo "alias resource=\"ros_source && psyche_source\"" >> $HOME/.bashrc
RUN echo "alias pbuild=\"here=$(pwd) && cd /psyche && colcon build && resource && cd '\$here'\"" >> $HOME/.bashrc

COPY . /psyche
RUN rm -rf /psyche/{build,install,log}
WORKDIR /psyche/src/ros2_mpu6050_driver
RUN patch -p1 < /psyche/mpu.patch
WORKDIR /psyche
RUN rosdep update 

RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y"
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && \
    colcon build"

CMD ["/psyche/launch.sh"]
