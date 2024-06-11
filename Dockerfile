ARG ROS_REPO=arm64v8/ros:iron
FROM ${ROS_REPO}

# Set up workspace and build
RUN mkdir -p /psyche
COPY . /psyche
WORKDIR /psyche
RUN rm -rf {build,install,log,memory}
RUN ./setup.sh

CMD ["/psyche/launch.sh"]
