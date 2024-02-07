FROM ubuntu:20.04

# Update package lists
RUN apt update -y

# Install software-properties-common
RUN apt install software-properties-common -y

# Add deadsnakes PPA repository and install Python 3.9
RUN add-apt-repository ppa:deadsnakes/ppa -y && \
    apt install python3.9 -y

# Set non-interactive frontend for Debian
ARG DEBIAN_FRONTEND=noninteractive

# Basic software installation
RUN apt install -y vim git


# Install Python dependencies
RUN apt install -y \
    python3-pip \
    python3-pyqt5.qtmultimedia \
    libqt5multimedia5-plugins \
    g++ \
    libblas-dev \
    liblapack-dev \
    libpng-dev \
    libfreetype6-dev \
    libjpeg-dev \
    zlib1g-dev \
    build-essential \
    cmake \
    libyaml-cpp-dev \
    libpcap-dev \
    libeigen3-dev \
    libjsoncpp-dev \
    dirmngr \
    gnupg2


RUN sh -c pip3 install --no-cache-dir -r ./requirements.txt
# Install ROS Noetic
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
    echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list && \
    apt update -y && \
    apt install -y --no-install-recommends ros-noetic-desktop-full && \
    rm -rf /var/lib/apt/lists/*

# Install ROS packages
RUN apt update -y && \
    apt install -y --no-install-recommends \
    ros-noetic-jsk-recognition-msgs \
    ros-noetic-jsk-rviz-plugins \
    ros-noetic-novatel-oem7-driver

# Set environment variables
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV ROS_DISTRO noetic

# Set working directory
WORKDIR /workspace

# Add custom commands to .bashrc
RUN echo "alias cdmobexcute='cd /workspace/src/mobinha/selfdrive/manager/'" >> /root/.bashrc && \
    echo "alias cdmob='cd /workspace/src/mobinha/'" >> /root/.bashrc && \
    echo "alias runmob='cd /workspace/src/mobinha/selfdrive/manager/ && ./execute_docker.sh'" >> /root/.bashrc

COPY ./src/docker/ /workspace/src/docker/

# Copy and run run.sh file
COPY ./run.sh /workspace/src/docker/run.sh
RUN chmod +x /workspace/src/docker/run.sh
CMD ["/workspace/src/docker/run.sh"]

