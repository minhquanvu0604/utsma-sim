FROM arm64v8/ros:noetic-ros-base

# Setup basic
RUN apt update && apt install -y \
    git \
    gcc \
    curl \
    wget \
    python3-catkin-tools \
    python3-pip \
    libudev-dev \
    qtbase5-dev \
    qtdeclarative5-dev \
    qtmultimedia5-dev \
    qtquickcontrols2-5-dev && \
    pip3 install flask waitress opencv-python numpy datetime

# Configure ur_ws
RUN mkdir -p utsma_ws/src && \
    cd utsma_ws && \
    catkin init && \
    echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc