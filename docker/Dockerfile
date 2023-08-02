ARG ROS_DISTRO=humble
ARG UNDERLAY_WS=/bot_mini_ws
ARG OVERLAY_WS=/bot_mini_ws

# This file should work for both amd64 and arm64 builds.

########################################
# Base Image for Bot Mini Control #
########################################
FROM ros:${ROS_DISTRO} as base
ENV ROS_DISTRO=${ROS_DISTRO}
ENV UNDERLAY_WS=${UNDERLAY_WS}

SHELL ["/bin/bash", "-c"]

# Create Colcon workspace with external dependencies
RUN mkdir -p /${UNDERLAY_WS}/src
WORKDIR /${UNDERLAY_WS}/src
COPY dependencies.repos .

#RUN vcs import < dependencies.repos
RUN vcs import < dependencies.repos;

# Build the base Colcon workspace, installing dependencies first.
WORKDIR /${UNDERLAY_WS}
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
 && apt-get update -y \
 && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y \
 && colcon build --symlink-install \
 && rm -rf /var/lib/apt/lists/* 

# Use Cyclone DDS as middleware
RUN apt-get update && apt-get install -y --no-install-recommends \
 ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
 ros-${ROS_DISTRO}-xacro \
 && rm -rf /var/lib/apt/lists/*

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

###########################################
# Overlay Image for Bot Mini Control #
###########################################
FROM base AS overlay

ENV OVERLAY_WS=${OVERLAY_WS}
 
# Create an overlay Colcon workspace
RUN mkdir -p /${OVERLAY_WS}/src
WORKDIR /${OVERLAY_WS}/src
COPY overlay.repos .
RUN vcs import < overlay.repos

WORKDIR /${OVERLAY_WS}

RUN source /${UNDERLAY_WS}/install/setup.bash \
 && colcon build --symlink-install \
 && apt-get update \
 && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y \
 && rm -rf /var/lib/apt/lists/*

# Set up the entrypoint
COPY ./docker/entrypoint.sh /
RUN chmod +x /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]

###########################################
# GUI Additions for Ros2 #
###########################################
FROM overlay AS guis

# Install additional GUI tools
RUN source /${UNDERLAY_WS}/install/setup.bash \
 && colcon build --symlink-install \
 && apt-get update \
 && apt-get install -y --no-install-recommends ros-${ROS_DISTRO}-rviz2 \
 && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y \
 && rm -rf /var/lib/apt/lists/*

# Set up the entrypoint
ENTRYPOINT [ "/entrypoint.sh" ]

LABEL org.opencontainers.image.source=https://github.com/bjoernellens1/cps_bot_mini_ws