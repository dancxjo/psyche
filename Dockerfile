# ROS 2 base container for the "auditor" host-like runtime.
#
# Build args:
# - ROS_DISTRO: Ros2 distro (e.g., jazzy, iron, humble). Default: jazzy
# - HOST_UID / HOST_GID: Host user/group id for dev volumes. Default: 1000/1000
#
# Example:
#   docker build \
#     --build-arg ROS_DISTRO=jazzy \
#     --build-arg HOST_UID=$(id -u) \
#     --build-arg HOST_GID=$(id -g) \
#     -t psyche/auditor:jazzy .

# Select distro at build time; default to jazzy to match hosts
ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base

ARG DEBIAN_FRONTEND=noninteractive
ARG ROS_DISTRO
ARG HOST_UID=1000
ARG HOST_GID=1000

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Basic tooling and colcon for building workspaces
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        locales tzdata \
        python3-pip python3-colcon-common-extensions \
        git bash-completion less vim \
    && rm -rf /var/lib/apt/lists/*

# Locale + timezone sane defaults
RUN locale-gen en_US.UTF-8 \
    && update-locale LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8 \
    TZ=UTC

# Create a non-root user matching host UID/GID for volume-permissions friendliness
RUN groupadd --gid ${HOST_GID} rosuser \
    && useradd --uid ${HOST_UID} --gid ${HOST_GID} -m -s /bin/bash rosuser

# Workspace directory (mounted via compose) and shell conveniences
ENV WORKDIR=/workspaces/psyche \
    ROS_WS=/workspaces/ros2_ws
RUN mkdir -p ${WORKDIR} ${ROS_WS} \
    && chown -R ${HOST_UID}:${HOST_GID} ${WORKDIR} ${ROS_WS}

# Add entrypoint that sources ROS and preserves user env
COPY scripts/ros_entrypoint.sh /usr/local/bin/ros_entrypoint.sh
RUN chmod +x /usr/local/bin/ros_entrypoint.sh

# Switch to non-root by default
USER rosuser
WORKDIR ${WORKDIR}

# Source ROS setup in interactive shells
ENV ROS_DISTRO=${ROS_DISTRO}
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc \
    && echo 'export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}' >> ~/.bashrc \
    && echo 'export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}' >> ~/.bashrc

ENTRYPOINT ["/usr/local/bin/ros_entrypoint.sh"]
CMD ["bash"]
