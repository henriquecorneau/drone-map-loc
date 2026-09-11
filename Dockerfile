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

RUN git clone https://github.com/natalnet-lpr/ORB_SLAM3_Modular.git

RUN git clone --recursive https://github.com/stevenlovegrove/Pangolin.git \
  && cd Pangolin/ \
  && sudo apt-get update \
  && sudo apt-get install -y python3-pip python3-setuptools python3-wheel \
  && yes | ./scripts/install_prerequisites.sh recommended \
  && cmake -B build -GNinja \
  && cmake --build build \
  && sudo cmake --install build \
  && sudo ldconfig

RUN cd ORB_SLAM3_Modular \
  && chmod +x build.sh \
  && ./build.sh

CMD ["bash"]
