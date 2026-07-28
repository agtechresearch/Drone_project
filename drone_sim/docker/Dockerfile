# Starling 2 SITL 개발 환경
# Base: Ubuntu 22.04 + PX4 v1.14.0 + Gazebo Classic 11 + ROS2 Humble
# Note: ros-humble-gazebo-ros-pkgs는 의도적으로 설치하지 않음 (SITL과 충돌)

FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8
ENV TZ=Asia/Seoul

# 기본 도구 및 로케일
RUN apt-get update && apt-get install -y --no-install-recommends \
    locales tzdata curl wget gnupg2 lsb-release ca-certificates \
    software-properties-common sudo git vim nano \
    build-essential cmake ninja-build ccache \
    python3 python3-pip python3-dev \
    && locale-gen en_US.UTF-8 \
    && ln -snf /usr/share/zoneinfo/$TZ /etc/localtime \
    && rm -rf /var/lib/apt/lists/*

# Gazebo Classic 11 (OSRF 저장소)
RUN wget -qO - https://packages.osrfoundation.org/gazebo.gpg | \
    gpg --dearmor -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=amd64 signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list && \
    apt-get update && apt-get install -y --no-install-recommends \
    gazebo libgazebo11 libgazebo-dev \
    && rm -rf /var/lib/apt/lists/*

# ROS2 Humble (gazebo-ros-pkgs 제외)
RUN add-apt-repository universe && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-base \
    ros-humble-rmw-fastrtps-cpp \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-argcomplete \
    && rm -rf /var/lib/apt/lists/*

# rosdep init
RUN rosdep init && rosdep update

# PX4 필수 의존성 (Tools/setup/ubuntu.sh 참고, --no-sim-tools 준용)
RUN apt-get update && apt-get install -y --no-install-recommends \
    astyle \
    bzip2 \
    dirmngr \
    file \
    gdb \
    genromfs \
    gcc-arm-none-eabi \
    gperf \
    lcov \
    libncurses5-dev \
    libtool \
    net-tools \
    ninja-build \
    patch \
    pkg-config \
    python3-dev \
    python3-pip \
    python3-setuptools \
    python3-wheel \
    rsync \
    shellcheck \
    unzip \
    xz-utils \
    zip \
    protobuf-compiler \
    libeigen3-dev \
    libopencv-dev \
    libgstreamer-plugins-base1.0-dev \
    && rm -rf /var/lib/apt/lists/*

# PX4 Python 패키지
RUN pip3 install --no-cache-dir \
    argcomplete argparse cerberus coverage empy==3.3.4 jinja2 \
    matplotlib numpy nunavut packaging pandas pkgconfig \
    pyros-genmsg pyserial pyulog>=0.5.0 pyyaml requests setuptools \
    six toml wheel jsonschema psutil pymavlink kconfiglib future \
    lxml==4.9.4

# MAVSDK-Python (0.12.0 = 실기와 동일)
RUN pip3 install --no-cache-dir mavsdk==0.12.0

# xvfb (GUI 없는 환경 대비, 하지만 이 컨테이너는 X11 forwarding으로 GUI 지원)
RUN apt-get update && apt-get install -y --no-install-recommends \
    xvfb x11-apps mesa-utils libgl1-mesa-glx libgl1-mesa-dri \
    && rm -rf /var/lib/apt/lists/*

# 사용자 계정 생성 (호스트 UID/GID와 맞춤 - 볼륨 마운트 권한 문제 방지)
ARG USER_UID=1000
ARG USER_GID=1000
ARG USERNAME=dronesim
RUN groupadd --gid ${USER_GID} ${USERNAME} && \
    useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} -s /bin/bash && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME}

# ROS2 setup을 사용자 bashrc에 (자동 source)
RUN echo "source /opt/ros/humble/setup.bash" >> /home/${USERNAME}/.bashrc && \
    chown ${USERNAME}:${USERNAME} /home/${USERNAME}/.bashrc

USER ${USERNAME}
WORKDIR /home/${USERNAME}

CMD ["/bin/bash"]
