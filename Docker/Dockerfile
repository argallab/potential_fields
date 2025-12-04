FROM ros:jazzy
ENV ROS_DISTRO jazzy

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-joy \
    ros-${ROS_DISTRO}-kinematics-interface-kdl \
    ros-${ROS_DISTRO}-ament-cmake-clang-format \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-moveit-ros-perception \
    ros-${ROS_DISTRO}-ackermann-msgs \
    ros-${ROS_DISTRO}-control-toolbox \
    ros-${ROS_DISTRO}-generate-parameter-library \
    ros-${ROS_DISTRO}-generate-parameter-library-py \
    ros-${ROS_DISTRO}-moveit-ros-control-interface \
    ros-${ROS_DISTRO}-pick-ik \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-tf2-eigen \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-trajectory-msgs \
    ros-${ROS_DISTRO}-geometric-shapes \
    ros-${ROS_DISTRO}-urdf \
    ros-${ROS_DISTRO}-gz-ros2-control \
    ros-${ROS_DISTRO}-ros-gz-sim \
    ros-${ROS_DISTRO}-pinocchio \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    libeigen3-dev \
    libfcl-dev \
    libccd-dev \
    libassimp-dev \
    liburdfdom-dev \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# install ubuntu packages
RUN apt-get update -yq && apt-get install -yq \
    python3 \
    apt-utils \
    locales \
    curl \
    gnupg \
    lsb-release \
    libx11-dev \
    libxau-dev \
    libxcb1-dev \
    libxdmcp-dev \
    libxext-dev \
    lsb-release \
    pkg-config \
    byobu \
    nano \
    python3-pip \
    ssh \
    vim \
    gdb \
    udev \
    python3-pyudev \
    ipython3 \
    mesa-utils \
    autoconf \
    automake \
    libtool \
    make \
    g++ \
    unzip \
    git \
    python3-scipy \
    python3-numpy \
    python3-cvxopt \
    python3-pandas \
    python3-colcon-common-extensions \
    python-is-python3 \
    python3-tk \
    python3-pytest-rerunfailures \
    python3-matplotlib \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Install necessary packages for libfranka
RUN apt-get update \
    && apt-get install -y \
    bash-completion \
    build-essential \
    cmake \
    doxygen \
    graphviz \
    lcov \
    libeigen3-dev \
    libpoco-dev \
    rename \
    valgrind \
    lsb-release \
    dpkg \
    libfmt-dev \
    python3-dev \
    liburdfdom-headers-dev \
    libconsole-bridge-dev \
    libtinyxml2-dev \
    lsb-release \
    wget \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*

# Install robotpkg
RUN mkdir -p /etc/apt/keyrings && \
    curl -fsSL http://robotpkg.openrobots.org/packages/debian/robotpkg.asc | tee /etc/apt/keyrings/robotpkg.asc && \
    echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" \
    | tee /etc/apt/sources.list.d/robotpkg.list && \
    apt-get update

# Install Pinocchio
RUN apt-get install -y robotpkg-pinocchio && \
    apt-get clean -y && \
    rm -rf /var/lib/apt/lists/*

# Set environment variables
ENV PATH=/opt/openrobots/bin:$PATH \
    PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH \
    LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH \
    PYTHONPATH=/opt/openrobots/lib/python3.10/site-packages:$PYTHONPATH \
    CMAKE_PREFIX_PATH=/opt/openrobots/lib/cmake:$CMAKE_PREFIX_PATH

# install xArm python sdk
RUN git clone https://github.com/xArm-Developer/xArm-Python-SDK.git && \
    cd xArm-Python-SDK && \
    pip install . --break-system-packages

# # Install EigenPy
# RUN apt-get update && \
#     apt install robotpkg-py35-eigenpy

# install example robot data for urdf models
# RUN git clone --recursive https://github.com/Gepetto/example-robot-data.git && \
#     cd example-robot-data && \
#     mkdir -p build && \
#     cd build && \
#     cmake .. && \
#     make && \
#     make install

# install box 2d
RUN pip3 install Box2D --break-system-packages

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

ENV QT_X11_NO_MITSHM=1

# add udev rules
RUN mkdir -p /etc/udev/rules.d
COPY ./udev_rules/* /etc/udev/rules.d/

# create a generic workplace directory
RUN mkdir -p /home/workspace/src
WORKDIR /home/workspace

# source ROS2
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc

# RUN chmod a+x entrypoint.sh
COPY ./entrypoint.sh /
COPY ./build_pfield_lib.sh /usr/local/bin/
RUN chmod a+x /entrypoint.sh && chmod a+x /usr/local/bin/build_pfield_lib.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]