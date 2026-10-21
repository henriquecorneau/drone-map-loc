FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive

ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=1000

RUN apt-get update && apt-get install -y \
    sudo \
    cmake \
    git \
    pkg-config \
    libpython3-dev \
    libegl1-mesa-dev \
    libgl1-mesa-dev \
    libglew-dev \
    libepoxy-dev \
    libxkbcommon-dev \
    libopencv-dev \
    libboost-all-dev \
    libssl-dev \
    ros-humble-foxglove-bridge \
    ros-humble-rviz2 \
    wget \
    && rm -rf /var/lib/apt/lists/*

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -ms /bin/bash --uid $USER_UID --gid $USER_GID $USERNAME \
    && usermod -aG sudo $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

USER $USERNAME
WORKDIR /home/$USERNAME

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

RUN git clone --recursive https://github.com/stevenlovegrove/Pangolin.git \
  && cd Pangolin/ \
  && sudo apt-get update \
  && sudo apt-get install -y python3-pip python3-setuptools python3-wheel \
  && yes | ./scripts/install_prerequisites.sh recommended \
  && cmake -B build -GNinja \
  && cmake --build build \
  && sudo cmake --install build \
  && sudo ldconfig

# Our fork of huynhduc9905/ORB_SLAM3 (adds relocalization in localization-only
# mode). Pinned so rebuilds are reproducible; bump to pick up fork changes.
ARG ORB_SLAM3_REPO=https://github.com/henriquecorneau/ORB_SLAM3.git
ARG ORB_SLAM3_COMMIT=99998f4dc8f94eb678eba36d8862cc5504226726

RUN git clone ${ORB_SLAM3_REPO} ORB_SLAM3 \
  && cd ORB_SLAM3 \
  && git checkout ${ORB_SLAM3_COMMIT} \
  && chmod +x build.sh \
  && ./build.sh

CMD ["bash"]
