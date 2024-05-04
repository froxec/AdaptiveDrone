FROM nvidia/cuda:11.8.0-runtime-ubuntu22.04 AS base

ENV DEBIAN_FRONTEND noninteractive

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
    pkg-config 

# install vcstool and colcon (used to build gazebo from source)
RUN pip install vcstool || pip3 install vcstool \
    && pip install -U colcon-common-extensions || \
    pip3 install -U colcon-common-extensions

# Create the non-root user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

FROM base as gazebo

# create workspace
RUN mkdir -p /workspace/src
WORKDIR /workspace/src

#########################
# install gazebo-garden 
#########################

# suppress known optix error
ENV SKIP_optix true

# get gazebo garden
RUN wget https://raw.githubusercontent.com/gazebo-tooling/gazebodistro/master/collection-garden.yaml \
    # retrieve lib sources
    && vcs import < collection-garden.yaml

# install dependencies
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
    http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
    && apt-get update \
    && apt -y install \
    $(sort -u $(find . -iname 'packages-'`lsb_release -cs`'.apt' -o -iname 'packages.apt' | grep -v '/\.git/') | sed '/gz\|sdf/d' | tr '\n' ' ')

# build gazebo libraries
WORKDIR /workspace
RUN colcon graph
RUN colcon build --merge-install

# source gazebo-garden
# COPY run_entrypoint.sh .
# ENTRYPOINT ["/workspace/run_entrypoint.sh"]