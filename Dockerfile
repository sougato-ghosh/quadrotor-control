FROM osrf/ros:humble-desktop-full

# Prevent interactive prompts during installation
ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies including ROS2 bridge, sound, GUI, python modules, and ALSA-to-Pulse plugins
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-pyqt5 \
    ros-humble-ros-gz-bridge \
    ros-humble-ros-gz-sim \
    ros-humble-ros-gz-interfaces \
    portaudio19-dev \
    pulseaudio \
    pulseaudio-utils \
    alsa-utils \
    libasound2-dev \
    libasound2-plugins \
    git \
    wget \
    unzip \
    && rm -rf /var/lib/apt/lists/*

# Configure ALSA to redirect default PCM and control devices to PulseAudio
RUN echo "pcm.!default { type pulse }" > /etc/asound.conf && \
    echo "ctl.!default { type pulse }" >> /etc/asound.conf

# Configure PulseAudio client inside the container to connect directly to the unix socket and disable shared memory / X11 lookup bypass
RUN mkdir -p /etc/pulse && \
    echo "default-server = unix:/run/user/1000/pulse/native" > /etc/pulse/client.conf && \
    echo "enable-shm = no" >> /etc/pulse/client.conf && \
    echo "auto-connect-display = no" >> /etc/pulse/client.conf && \
    echo "auto-connect-localhost = no" >> /etc/pulse/client.conf

RUN mkdir -p /root/.config/pulse && \
    echo "default-server = unix:/run/user/1000/pulse/native" > /root/.config/pulse/client.conf && \
    echo "enable-shm = no" >> /root/.config/pulse/client.conf && \
    echo "auto-connect-display = no" >> /root/.config/pulse/client.conf && \
    echo "auto-connect-localhost = no" >> /root/.config/pulse/client.conf

# Install python packages for Vosk offline speech recognition and sound device
RUN pip3 install vosk sounddevice numpy

# Set up local Vosk voice model directory and download the small US English model
RUN mkdir -p /opt/vosk-model && \
    wget -qO /tmp/vosk-model.zip https://alphacephei.com/vosk/models/vosk-model-small-en-us-0.15.zip && \
    unzip -q /tmp/vosk-model.zip -d /tmp && \
    mv /tmp/vosk-model-small-en-us-0.15/* /opt/vosk-model/ && \
    rm -rf /tmp/vosk-model-small-en-us-0.15 /tmp/vosk-model.zip

# Pre-cache X3 UAV Gazebo/Ignition fuel model so it runs 100% offline at runtime
RUN mkdir -p /root/.ignition/fuel/fuel.ignitionrobotics.org/openrobotics/models/x3\ uav/4 && \
    wget -qO /tmp/model.sdf "https://fuel.gazebosim.org/1.0/OpenRobotics/models/X3%20UAV/tip/files/model.sdf" && \
    wget -qO /tmp/model.config "https://fuel.gazebosim.org/1.0/OpenRobotics/models/X3%20UAV/tip/files/model.config" && \
    mkdir -p /tmp/meshes && \
    wget -qO /tmp/meshes/x3.dae "https://fuel.gazebosim.org/1.0/OpenRobotics/models/X3%20UAV/tip/files/meshes/x3.dae" && \
    wget -qO /tmp/meshes/x3.jpg "https://fuel.gazebosim.org/1.0/OpenRobotics/models/X3%20UAV/tip/files/meshes/x3.jpg" && \
    wget -qO /tmp/meshes/propeller_ccw.dae "https://fuel.gazebosim.org/1.0/OpenRobotics/models/X3%20UAV/tip/files/meshes/propeller_ccw.dae" && \
    wget -qO /tmp/meshes/propeller_cw.dae "https://fuel.gazebosim.org/1.0/OpenRobotics/models/X3%20UAV/tip/files/meshes/propeller_cw.dae" && \
    # Move them into the expected ignition cache structure
    mv /tmp/model.sdf /root/.ignition/fuel/fuel.ignitionrobotics.org/openrobotics/models/x3\ uav/4/model.sdf && \
    mv /tmp/model.config /root/.ignition/fuel/fuel.ignitionrobotics.org/openrobotics/models/x3\ uav/4/model.config && \
    mkdir -p /root/.ignition/fuel/fuel.ignitionrobotics.org/openrobotics/models/x3\ uav/4/meshes && \
    mv /tmp/meshes/* /root/.ignition/fuel/fuel.ignitionrobotics.org/openrobotics/models/x3\ uav/4/meshes/ && \
    # Modify the fuel model's internal paths if needed (it already points to model://x3/meshes/... or similar)
    echo "Cached X3 UAV successfully"

# Create symlinks to ensure offline model and mesh resolution
RUN ln -s "/root/.ignition/fuel/fuel.ignitionrobotics.org/openrobotics/models/x3 uav/4" "/root/.ignition/fuel/fuel.ignitionrobotics.org/openrobotics/models/x3" && \
    ln -s "/root/.ignition/fuel/fuel.ignitionrobotics.org/openrobotics/models/x3 uav/4" "/root/.ignition/fuel/fuel.ignitionrobotics.org/openrobotics/models/X3" && \
    ln -s "/root/.ignition/fuel/fuel.ignitionrobotics.org/openrobotics/models/x3 uav" "/root/.ignition/fuel/fuel.ignitionrobotics.org/openrobotics/models/X3 UAV"

# Set Gazebo/Ignition local resource/model search paths
ENV IGN_GAZEBO_RESOURCE_PATH=/root/.ignition/fuel/fuel.ignitionrobotics.org/openrobotics/models
ENV SDF_PATH=/root/.ignition/fuel/fuel.ignitionrobotics.org/openrobotics/models

# Create a workspace
RUN mkdir -p /workspace/src
WORKDIR /workspace

# Copy the local package
COPY src/ /workspace/src/

# Source ROS2 and build the package
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"

# Add setup to bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /workspace/install/setup.bash" >> /root/.bashrc

# Copy entrypoint script
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "quadrotor_control", "main.launch.py"]
