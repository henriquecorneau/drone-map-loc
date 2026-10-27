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
# mode and snapshots of every atlas map). Pinned so rebuilds are reproducible;
# bump to pick up fork changes.
ARG ORB_SLAM3_REPO=https://github.com/henriquecorneau/ORB_SLAM3.git
ARG ORB_SLAM3_COMMIT=8affde83c0afadb0984cf0910844e613ac3c04c7

RUN git clone ${ORB_SLAM3_REPO} ORB_SLAM3 \
  && cd ORB_SLAM3 \
  && git checkout ${ORB_SLAM3_COMMIT} \
  && chmod +x build.sh \
  && ./build.sh

# Tello driver (ros2_ws/src/tello_driver): tello-zune pinned to the commit of
# zunedrones/tello_zune#34 (get_frame_stamped, get_state, rc watchdog); switch
# to the PyPI release once it is published. --no-deps keeps the apt OpenCV and
# NumPy 1.x that Humble's cv_bridge was built against.
ARG TELLO_ZUNE_REF=61a411e8e443bb6390f1919fa09057be2193a7c5

RUN sudo apt-get update \
  && sudo apt-get install -y python3-opencv python3-scipy python3-pytest \
     ros-humble-cv-bridge ros-humble-diagnostic-msgs \
  && sudo rm -rf /var/lib/apt/lists/* \
  && python3 -m pip install --user --no-cache-dir --no-deps \
     "tello_zune @ git+https://github.com/zunedrones/tello_zune.git@${TELLO_ZUNE_REF}"

CMD ["bash"]
