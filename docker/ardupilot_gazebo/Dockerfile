FROM nvidia/cuda:11.8.0-runtime-ubuntu22.04 AS base

ARG DEBIAN_FRONTEND=noninteractive

ARG USERNAME=simuser
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# setup locales
RUN apt-get update && apt-get install -y locales \
    && sed -i '/en_US.UTF-8/s/^# //g' /etc/locale.gen \
    && locale-gen
ENV LANG en_US.UTF-8  
ENV LANGUAGE en_US:en  
ENV LC_ALL en_US.UTF-8
ENV TZ Etc/UTC

# install dependencies
RUN apt install -y \
    python3-pip \
    wget \
    lsb-release \
    curl \
    gnupg \
    git \
    libegl1-mesa-dev \
    mesa-utils \
    cmake \
    pkg-config \
    net-tools \
    tzdata

# install vcstool and colcon (used to build gazebo from source)
RUN pip install vcstool || pip3 install vcstool \
    && pip install -U colcon-common-extensions || \
    pip3 install -U colcon-common-extensions

# Create the non-root user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID -s /bin/bash --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

FROM base as gazebo

# create workspace
RUN mkdir -p /workspace/src
WORKDIR /workspace/src

#########################
# install gazebo-harmonic
#########################

# suppress known optix error
ENV SKIP_optix true

# get gazebo harmonic
RUN wget https://raw.githubusercontent.com/gazebo-tooling/gazebodistro/master/collection-harmonic.yaml \
    # retrieve lib sources
    && vcs import < collection-harmonic.yaml

# install dependencies
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
    && apt-get update \
    && apt-get install -y gz-harmonic

###########################################
# Build ardupilot_gazebo for gazebo harmonic
###########################################

FROM gazebo as ardupilot_gazebo
# install dependencies
RUN apt update && apt install -y \
    libgz-sim8-dev \
    rapidjson-dev

# create workspace
RUN mkdir -p /workspace/ardupilot_gazebo_ws/src/
WORKDIR /workspace/ardupilot_gazebo_ws/src
RUN git clone https://github.com/ArduPilot/ardupilot_gazebo

# build plugin
ENV GZ_VERSION harmonic
RUN cd ardupilot_gazebo \
    && mkdir build && cd build \
    && cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    && make -j4

###########################################
# Build ardupilot sitl
###########################################

FROM ardupilot_gazebo as ardupilot_sitl_gazebo
ENV SKIP_AP_GIT_CHECK 1
RUN mkdir -p /workspace/ardupilot_sitl/src/
RUN chown $USERNAME /workspace/ardupilot_sitl/src/
RUN chown $USERNAME /
USER $USERNAME
WORKDIR /workspace/ardupilot_sitl/src/
RUN git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git 
WORKDIR /workspace/ardupilot_sitl/src/ardupilot
# set $USER for install-prereqs-ubuntu.sh
ENV USER $USERNAME
ENV DEBIAN_FRONTEND noninteractive
RUN export DEBIAN_FRONTEND="noninteractive" \ 
    && ./Tools/environment_install/install-prereqs-ubuntu.sh -y

RUN python -m pip install empy pexpect future

# build models
RUN ./waf distclean
RUN ./waf configure --board sitl
RUN ./waf copter
RUN ./waf rover 
RUN ./waf plane
RUN ./waf sub

# SETUP GPU (gazebo lagging without it)
ENV NVIDIA_DRIVER_CAPABILITIES all
ENV NVIDIA_VISIBLE_DEVICES all
# configure shell
COPY run_entrypoint.bash /home/${USER}/
RUN sudo chmod +x /home/${USER}/run_entrypoint.bash
#ENTRYPOINT "/home/${USER}/run_entrypoint.bash"