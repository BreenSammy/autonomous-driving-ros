FROM osrf/ros:noetic-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# Install as root
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-catkin-tools \
    git-all \
    && rm -rf /var/lib/apt/lists/*

# Create ros user
ARG USER_ID=1000
ARG GROUP_ID=1000
RUN groupadd -g ${GROUP_ID} ros \
    && useradd -m -u ${USER_ID} -g ${GROUP_ID} -s /bin/bash ros \
    && usermod -aG sudo,dialout ros \
    && echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

# Set environment for username after user is created
ENV USERNAME=ros
USER ros

# Configure shell and update rosdep
RUN echo "source /opt/ros/\$ROS_DISTRO/setup.bash" >> /home/$USERNAME/.bashrc \
    && rosdep update

# Create workspace and copy source
WORKDIR /home/$USERNAME/workspace
COPY --chown=ros ./src src

RUN source /opt/ros/$ROS_DISTRO/setup.bash \
    && sudo rosdep install --from-paths . --ignore-src -r -y --rosdistro=$ROS_DISTRO \
    && echo 'source ~/workspace/install/local_setup.bash' >> /home/$USERNAME/.bashrc
