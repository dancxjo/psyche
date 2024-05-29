FROM ros:iron

RUN apt-get update && \
    apt-get upgrade -y 

RUN apt-get install -y \
    python3-pip \
    ros-iron-rmw-cyclonedds-cpp \
    ros-dev-tools \
    python3-colcon-common-extensions \
    ros-iron-diagnostic-updater 

# Convenience tools
RUN apt-get install -y \
    screen \
    micro \
    mc \
    nano

RUN rosdep update && \
    apt-get update && \
    apt-get upgrade -y

# Install Python packages
RUN pip install -qU \
    langchain \
    langchain-community \
    langchain-openai \
    sentence_splitter

# Set up workspace and build
RUN mkdir -p /psyche
#RUN git clone https://github.com/dancxjo/psyche.git /psyche
#Until we have a public repo
COPY . /psyche

WORKDIR /psyche
RUN /bin/bash -c "source /opt/ros/iron/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y"
RUN /bin/bash -c "source /opt/ros/iron/setup.bash && \
    colcon build"

RUN echo "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /etc/environment 
RUN echo "ROS_DOMAIN_ID=42" >> /etc/environment
RUN echo "source /opt/ros/iron/setup.bash" >> /etc/bash.bashrc
RUN echo "source /psyche/install/setup.bash" >> /etc/bash.bashrc

RUN sed -e "s/\bexec\b/\/bin\/bash -c/g" /ros_entrypoint.sh > /ros_entrypoint.sh

CMD ["/psyche/launch.sh"]
