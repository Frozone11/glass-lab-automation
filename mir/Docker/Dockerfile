FROM osrf/ros:noetic-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

# Install tools required to build the MiR ROS packages.
RUN apt-get update && apt-get install -y \
    git \
    python3-rosdep \
    python3-catkin-tools \
    python3-pip \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep.
RUN rosdep init || true
RUN rosdep update --include-eol-distros

# Create the ROS workspace.
RUN mkdir -p /root/catkin_ws/src

# Download the DFKI MiR repository, Noetic branch.
RUN git clone --depth 1 -b noetic \
    https://github.com/DFKI-NI/mir_robot.git \
    /root/catkin_ws/src/mir_robot

# Install dependencies.
RUN apt-get update && \
    cd /root/catkin_ws && \
    rosdep install \
        --from-paths src \
        --ignore-src \
        --rosdistro noetic \
        -r -y && \
    rm -rf /var/lib/apt/lists/*

# Build the ROS workspace.
RUN /bin/bash -c \
    "source /opt/ros/noetic/setup.bash && \
     cd /root/catkin_ws && \
     catkin_make"

# Configure the ROS environment.
RUN echo 'source /opt/ros/noetic/setup.bash' >> /root/.bashrc && \
    echo 'source /root/catkin_ws/devel/setup.bash' >> /root/.bashrc

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

WORKDIR /root/catkin_ws

ENTRYPOINT ["/entrypoint.sh"]

CMD ["bash"]