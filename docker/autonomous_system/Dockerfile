FROM ros:humble-ros-base as base

ARG USERNAME=rosdev
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Install some dependencies packages
RUN apt-get update -q \
    && apt upgrade -q -y \
    && apt install -y --no-install-recommends \
    software-properties-common \
    python3-pip \
    xauth \
    net-tools \
    && apt clean \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# Install mavros
RUN apt-get update -q \
    && apt upgrade -q -y \
    && apt install -y --no-install-recommends \
    ros-humble-mavros \
    ros-humble-mavros-extras \
    && apt clean \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/* 

RUN wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh \
    && chmod a+x install_geographiclib_datasets.sh \ 
    && sudo ./install_geographiclib_datasets.sh

# Install rqt tools (GUI development)
RUN apt-get update -q \
    && apt install -y --no-install-recommends \
    ros-humble-rqt* \
    && apt clean \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/* 

# Create the non-root user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID -s /bin/bash --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Create workspace so that user own this directory
RUN mkdir -p /home/$USERNAME/ws/
WORKDIR /home/$USERNAME/ws

# Copy configuration files
RUN echo 'source /opt/ros/'$ROS_DISTRO'/setup.bash' >> /home/$USERNAME/.bashrc \
    && echo 'source /home/'$USERNAME'/ws/install/setup.bash' >> /home/$USERNAME/.bashrc
RUN chown -R rosdev /home/$USERNAME/ws/
