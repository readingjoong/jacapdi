# Ubuntu 22.04 (jammy) + ROS 2 Humble desktop + Gazebo Classic 11 + Python 3.10
FROM osrf/ros:humble-desktop

ARG USERNAME=sean
ARG USER_UID=1000
ARG USER_GID=1000
ARG TORCH_INDEX=https://download.pytorch.org/whl/cu126
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
      gazebo ros-humble-gazebo-ros-pkgs ros-humble-cv-bridge \
      ros-humble-ackermann-msgs ros-humble-tf-transformations \
      ros-dev-tools python3-pip python3-venv python3-colcon-common-extensions \
      python3-rosdep python3-serial python3-scipy python3-sklearn \
      build-essential ripgrep v4l-utils mesa-utils sudo less nano \
    && rm -rf /var/lib/apt/lists/*

# Same user name/uid as the host so /home/sean/team_ws keeps the pyc loader path depth.
RUN groupadd -g ${USER_GID} ${USERNAME} \
    && useradd -m -u ${USER_UID} -g ${USER_GID} -s /bin/bash ${USERNAME} \
    && usermod -aG video,dialout,plugdev ${USERNAME} \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME}

USER ${USERNAME}
WORKDIR /home/${USERNAME}

# venv on the system Python 3.10 (same interpreter as ROS binaries); PyTorch first.
RUN /usr/bin/python3 -m venv --system-site-packages /home/${USERNAME}/.venvs/jacapdi \
    && /home/${USERNAME}/.venvs/jacapdi/bin/python -m pip install --no-cache-dir -U pip \
    && /home/${USERNAME}/.venvs/jacapdi/bin/python -m pip install --no-cache-dir \
         torch torchvision --index-url ${TORCH_INDEX} \
    && /home/${USERNAME}/.venvs/jacapdi/bin/python -m pip install --no-cache-dir \
         'numpy==1.26.4' 'opencv-python==4.11.0.86' 'ultralytics==8.4.146' \
         pyserial 'scipy>=1.10,<1.16' 'scikit-learn>=1.3,<1.8'

RUN rosdep update --rosdistro humble \
    && echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc \
    && echo '[ -f ~/team_ws/install/setup.bash ] && source ~/team_ws/install/setup.bash' >> ~/.bashrc \
    && echo 'export ROS_LOCALHOST_ONLY=1' >> ~/.bashrc

ENV NVIDIA_VISIBLE_DEVICES=all NVIDIA_DRIVER_CAPABILITIES=all
CMD ["bash"]
