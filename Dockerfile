FROM ros:noetic-perception-focal
RUN apt update
RUN /bin/bash -c "echo 'source /opt/ros/noetic/setup.bash' >> ~/.bashrc && source ~/.bashrc"
RUN apt install -y ros-noetic-rviz \
                   ros-noetic-jsk-recognition-msgs \
                   ros-noetic-jsk-rviz-plugins \
                   ros-noetic-velodyne-pointcloud \
                   ros-noetic-teleop-twist-keyboard \
                   python3-pip \
                   python3-pcl \
                   python3-smbus \
                   libgstreamer1.0-dev \
                   gstreamer1.0-tools \
                   libgstreamer-plugins-base1.0-dev \
                   libgstreamer-plugins-good1.0-dev

RUN pip3 install -U pip
WORKDIR /home/ros_ws

