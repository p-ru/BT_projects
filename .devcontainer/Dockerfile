FROM ros:noetic

ARG DEBIAN_FRONTEND=noninteractive

# Create non-root user matching host UID
ARG USERNAME=matt
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME -s /bin/bash \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Install system dependencies
RUN apt-get update && apt-get install -y \
    libfmt-dev \
    git \
    ros-noetic-behaviortree-cpp-v3 \
    qtbase5-dev libqt5svg5-dev libzmq3-dev libdw-dev \
    ros-noetic-rqt \
    ros-noetic-rqt-common-plugins \
    ros-noetic-rqt-robot-plugins \
    ros-noetic-image-transport-plugins \
    ros-noetic-rosbridge-suite \
    ros-noetic-web-video-server \

    && rm -rf /var/lib/apt/lists/*

# Configure ROS environment for user
USER $USERNAME
RUN echo 'source /opt/ros/noetic/setup.bash' >> ~/.bashrc \
    && echo '[ -f /workspace/devel/setup.bash ] && source /workspace/devel/setup.bash' >> ~/.bashrc

WORKDIR /workspace
