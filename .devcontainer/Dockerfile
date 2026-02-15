# Start with ROS2 Humble base that already has CUDA support
FROM osrf/ros:humble-desktop-full

# Add ubuntu user with same UID and GID as your host system, if it doesn't already exist
ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN if ! id -u $USER_UID >/dev/null 2>&1; then \
        groupadd --gid $USER_GID $USERNAME && \
        useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME; \
    fi

# Add sudo support for the non-root user
RUN apt-get update && \
    apt-get install -y sudo && \
    echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

# Install NVIDIA CUDA toolkit, pip, and other dependencies
RUN apt-get update && apt-get install -y \
    nvidia-cuda-toolkit \
    python3-pip \
    git \
    ros-dev-tools \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    python-is-python3 \
    vim \
    tmux \
    && rm -rf /var/lib/apt/lists/*

# Switch from root to user
USER $USERNAME

# Add user to video group to allow access to webcam
RUN sudo usermod --append --groups video $USERNAME

# Rosdep update
RUN rosdep update

# Install PyTorch with CUDA support
RUN pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118

# Install numpy with a version less than 2 to avoid compatibility issues with some ROS packages
RUN pip3 install "numpy<2"

# Install YOLO dependencies
RUN pip3 install ultralytics

# Source the ROS setup file
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
RUN echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc
RUN echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> ~/.bashrc
RUN echo "export ROS_DOMAIN_ID=67 # change to your number" >> ~/.bashrc
RUN echo "export LIBGL_ALWAYS_SOFTWARE=1" >> ~/.bashrc

CMD ["bash"]