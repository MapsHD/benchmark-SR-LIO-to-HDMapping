FROM ubuntu:20.04

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC

# ── Base tools ────────────────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common \
    build-essential \
    cmake \
    git \
    apt-transport-https \
    ca-certificates \
    wget \
    libeigen3-dev \
    libboost-all-dev \
    libomp-dev \
    libtbb-dev \
    libpcl-dev \
    libopencv-dev \
    libgoogle-glog-dev \
    libgflags-dev \
    libyaml-cpp-dev \
    nlohmann-json3-dev \
    tmux \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# ── ROS 1 Noetic ─────────────────────────────────────────────────────────────
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros/ubuntu $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/ros.list && \
    apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop-full \
    ros-noetic-tf \
    ros-noetic-tf-conversions \
    ros-noetic-eigen-conversions \
    ros-noetic-pcl-conversions \
    ros-noetic-pcl-ros \
    ros-noetic-message-filters \
    ros-noetic-rosbag \
    ros-noetic-rosbag-storage \
    libceres-dev \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    python3-catkin-tools \
    && rm -rf /var/lib/apt/lists/*

# ── rosbags (used to convert ROS 2 bags to ROS 1, if needed) ─────────────────
RUN pip3 install --no-cache-dir "rosbags==0.9.22"

# ── Build catkin workspace (SR-LIO + converter) ──────────────────────────────
WORKDIR /ros_ws

COPY ./src/sr_lio             ./src/sr_lio
COPY ./src/srlio-to-hdmapping ./src/srlio-to-hdmapping

# Benchmark launch (selectable config, no velodyne conversion node) and extra
# configs (e.g. bunker = Livox Mid-360 as PointCloud2) added into the sr_lio
# package. SR-LIO writes debug output to /tmp/sr_lio_output.
COPY ./overlay/launch/ ./src/sr_lio/launch/
COPY ./overlay/config/ ./src/sr_lio/config/
RUN mkdir -p /tmp/sr_lio_output/cut_sweep /tmp/sr_lio_output/cloud_frame

# SR-LIO's CMakeLists compiles with -std=c++11, but PCL 1.10 (Noetic) headers
# require C++14 (POINT_CLOUD_REGISTER_POINT_STRUCT / point_representation.h fail
# under C++11 with "NrDims is not usable in a constant expression"). Bump it.
RUN sed -i 's/-std=c++11/-std=c++14/g; s/CMAKE_CXX_STANDARD 11/CMAKE_CXX_STANDARD 14/g' \
        src/sr_lio/CMakeLists.txt && \
    grep -q 'c++14' src/sr_lio/CMakeLists.txt

# Normalize the IMU accelerometer to m/s^2 (THE root-cause fix). The Bunker DVI
# /livox/imu stream reports linear_acceleration in units of g (~1.0 at rest);
# SR-LIO expects m/s^2 (~9.8) and, unlike FAST-LIO, never rescales internally.
# A g-unit stream therefore (a) inflates the init noise check by the
# (G_norm/|acc|)^2 ~ 96x term -> "Too large noise of accelerometer measurements",
# and (b) leaves an ~8.8 m/s^2 phantom acceleration -> the trajectory diverges /
# spirals. Inject an auto-scaling snippet into imuHandler; the scale is decided
# once from the first sample (|acc| < 3 => stream is in g => x 9.80665), so
# datasets already in m/s^2 get scale 1.0 and are unaffected.
COPY ./overlay/patches/ /ros_ws/patches/
RUN sed -i '/Imu::Ptr msg_temp(new/r /ros_ws/patches/imu_acc_scale.inc' \
        src/sr_lio/src/lioOptimization.cpp && \
    grep -q 'benchmark patch: normalize accelerometer' src/sr_lio/src/lioOptimization.cpp

# Init window: keep a short 0.5 s window so init completes during the brief
# static hold at the very start of the recording (before walking). With the
# accelerometer now in correct m/s^2 units the (G_norm/|acc|)^2 inflation is
# gone and these gates behave normally; the modestly relaxed bounds just tolerate
# minor hand tremor during the hold. Gravity is left as upstream (derived from
# the mean accelerometer) so it correctly captures the ~15 deg IMU mount tilt.
RUN sed -i \
        -e 's/#define MIN_INI_TIME .*/#define MIN_INI_TIME 0.5/' \
        -e 's/#define MAX_ACC_VAR .*/#define MAX_ACC_VAR 1.0/' \
        -e 's/#define MAX_GYR_VAR .*/#define MAX_GYR_VAR 1.0/' \
        src/sr_lio/include/utility.h && \
    grep -q 'MIN_INI_TIME 0.5' src/sr_lio/include/utility.h

# Build SR-LIO (lio_optimization) and the converter. The final guard fails the
# image build loudly if either executable is missing.
RUN source /opt/ros/noetic/setup.bash && \
    catkin_make -DCMAKE_BUILD_TYPE=Release -j$(nproc) && \
    test -f /ros_ws/devel/lib/sr_lio/lio_optimization && \
    test -f /ros_ws/devel/lib/srlio_to_hdmapping/listener && \
    echo "[build] sr_lio node and converter present"

# ── Non-root user ─────────────────────────────────────────────────────────────
ARG UID=1000
ARG GID=1000
RUN groupadd -g $GID ros && \
    useradd -m -u $UID -g $GID -s /bin/bash ros && \
    chown -R $UID:$GID /ros_ws && \
    chmod -R 777 /tmp/sr_lio_output

RUN echo "source /opt/ros/noetic/setup.bash"   >> /root/.bashrc && \
    echo "source /ros_ws/devel/setup.bash"     >> /root/.bashrc && \
    echo "source /opt/ros/noetic/setup.bash"   >> /home/ros/.bashrc && \
    echo "source /ros_ws/devel/setup.bash"     >> /home/ros/.bashrc

CMD ["bash"]
