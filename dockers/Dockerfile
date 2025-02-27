# =========================================================
# Builder Stage - ARM64 交叉编译
# =========================================================
ARG BUILDPLATFORM=linux/arm64
FROM --platform=${BUILDPLATFORM} public.ecr.aws/docker/library/ros:humble-ros-base AS builder

ARG TARGETARCH

# =========================================================
# 1. 设置交叉编译环境
# =========================================================
# 创建交叉编译工具链文件
RUN echo 'set(CMAKE_SYSTEM_NAME Linux)' > /ros2_ws/arm64_toolchain.cmake && \
    echo 'set(CMAKE_SYSTEM_PROCESSOR aarch64)' >> /ros2_ws/arm64_toolchain.cmake && \
    echo 'set(CMAKE_C_COMPILER aarch64-linux-gnu-gcc)' >> /ros2_ws/arm64_toolchain.cmake && \
    echo 'set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)' >> /ros2_ws/arm64_toolchain.cmake && \
    echo 'set(CMAKE_FIND_ROOT_PATH /usr/aarch64-linux-gnu /usr/lib/aarch64-linux-gnu /opt/ros/humble /opt/ros/humble/lib/aarch64-linux-gnu)' >> /ros2_ws/arm64_toolchain.cmake && \
    echo 'set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)' >> /ros2_ws/arm64_toolchain.cmake && \
    echo 'set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)' >> /ros2_ws/arm64_toolchain.cmake && \
    echo 'set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)' >> /ros2_ws/arm64_toolchain.cmake && \
    echo 'set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)' >> /ros2_ws/arm64_toolchain.cmake && \
    echo 'set(PKG_CONFIG_EXECUTABLE "/usr/bin/pkg-config")' >> /ros2_ws/arm64_toolchain.cmake

# 为交叉编译配置 PKG_CONFIG
RUN mkdir -p /usr/share/pkg-config-crossfile && \
    echo 'PKG_CONFIG_PATH="/usr/lib/aarch64-linux-gnu/pkgconfig:/opt/ros/humble/lib/aarch64-linux-gnu/pkgconfig"' >> /etc/environment && \
    echo 'PKG_CONFIG_LIBDIR="/usr/lib/aarch64-linux-gnu/pkgconfig:/opt/ros/humble/lib/aarch64-linux-gnu/pkgconfig"' >> /etc/environment && \
    echo "export CV_BRIDGE_DIR=/opt/ros/humble/share/cv_bridge/cmake" >> /etc/environment && \
    echo "export CV_BRIDGE_LIB_DIR=/opt/ros/humble/lib" >> /etc/environment

# =========================================================
# 2. 安装构建依赖项
# =========================================================
RUN apt-get update && \
    apt-get install -y \
        # 基本构建工具
        build-essential \
        cmake \
        pkg-config \
        # ROS 构建工具
        python3-colcon-common-extensions \
        python3-pip \
        python3-rosdep \
        # 交叉编译工具
        gcc-aarch64-linux-gnu \
        g++-aarch64-linux-gnu \
        qemu-user-static \
        binfmt-support && \
    # 为 ARM64 设置多架构支持
    dpkg --add-architecture arm64 && \
    apt-get update

# =========================================================
# 3. 安装 ARM64 依赖项
# =========================================================
RUN apt-get install -y --no-install-recommends \
        # 核心库
        libglib2.0-dev:arm64 \
        libboost-all-dev:arm64 \
        libeigen3-dev:arm64 \
        libcurl4-openssl-dev:arm64 \
        libjsoncpp-dev:arm64 \
        # 计算机视觉包
        ros-humble-cv-bridge:arm64 \
        ros-humble-vision-opencv:arm64 \
        libopencv-dev:arm64 \
        python3-cv-bridge:arm64 \
        ros-humble-vision-msgs:arm64 \
        # 相机包
        ros-humble-camera-info-manager:arm64 \
        ros-humble-image-transport:arm64 \
        # GStreamer 包
        libgstreamer1.0-dev:arm64 \
        libgstreamer-plugins-base1.0-dev:arm64 \
        libgstreamer-plugins-bad1.0-dev:arm64 \
        gstreamer1.0-plugins-base:arm64 \
        gstreamer1.0-plugins-good:arm64 \
        gstreamer1.0-plugins-bad:arm64 \
        # 包配置
        pkg-config:arm64 && \
    # 初始化 rosdep
    (rosdep init || echo "rosdep already initialized") && \
    rosdep update --rosdistro humble



FROM public.ecr.aws/docker/library/ros:humble-ros-base

# 安装运行时依赖项
RUN apt-get update && \
    dpkg --add-architecture arm64 && \
    apt-get update && \
    apt-get install -y \
    # 视觉和 OpenCV 包
    ros-humble-cv-bridge:arm64 \
    ros-humble-vision-opencv:arm64 \
    libopencv-dev:arm64 \
    python3-cv-bridge:arm64 \
    ros-humble-vision-msgs:arm64 \
    # Python 依赖项
    python3-pip \
    python3-dev \
    python3-numpy \
    # 相机和媒体包
    ros-humble-camera-info-manager:arm64 \
    ros-humble-image-transport:arm64 \
    gstreamer1.0-plugins-base:arm64 \
    gstreamer1.0-plugins-good:arm64 \
    gstreamer1.0-plugins-bad:arm64 \
    # 其他依赖项
    libeigen3-dev:arm64 && \
    # 安装 Python 包
    pip3 install numpy==1.23.5 ultralytics && \
    # 额外依赖项
    apt-get update && apt-get install -y \
        libgtk-3-dev:arm64 \
        libyaml-cpp-dev:arm64 \
        libprotobuf-dev:arm64 \
        protobuf-compiler:arm64 \
        libfftw3-dev:arm64 \
        libspdlog-dev:arm64 \
        libjpeg-dev:arm64 \
        libpng-dev:arm64 \
        libxml2-dev:arm64 \
        libssl-dev:arm64 \
        libtinyxml2-dev:arm64 \
        libzmq3-dev:arm64 \
        libglib2.0-dev \
        gstreamer1.0-tools \
        libgstreamer1.0-dev \
        libgstreamer-plugins-base1.0-dev \
        libgstreamer-plugins-bad1.0-dev \
        gstreamer1.0-plugins-base \
        gstreamer1.0-plugins-good \
        gstreamer1.0-plugins-bad \
        gstreamer1.0-plugins-ugly \
        gstreamer1.0-libav \
        libjsoncpp-dev \
        libcurl4-openssl-dev
