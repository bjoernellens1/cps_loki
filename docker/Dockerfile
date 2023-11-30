ARG ROS_DISTRO=humble
ARG UNDERLAY_WS=bot_mini_ws
ARG OVERLAY_WS=overlay_ws

# This file should work for both amd64 and arm64 builds.

## Problem! some variables not working currently. Need to find out why.
# For now, the variables are replaced with fixed values.

########################################
# Base Image for Bot Mini Control #
########################################
FROM ros:${ROS_DISTRO} as base

SHELL ["/bin/bash", "-c"]

ENV ROS_DISTRO=${ROS_DISTRO}
ENV UNDERLAY_WS=${UNDERLAY_WS}
ENV OVERLAY_WS=${OVERLAY_WS}

RUN echo "successfully sourced ENV variables: ROS_DISTRO=${ROS_DISTRO}, UNDERLAY_WS=${UNDERLAY_WS}, OVERLAY_WS=${OVERLAY_WS}"

# Create Colcon workspace with external dependencies
RUN mkdir -p /bot_mini_ws/src
WORKDIR /bot_mini_ws/src
COPY dependencies.repos .

#RUN vcs import < dependencies.repos
RUN vcs import < dependencies.repos;

# Build the base Colcon workspace, installing dependencies first.
WORKDIR /bot_mini_ws
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
 
# Create an overlay Colcon workspace
RUN mkdir -p /overlay_ws/src
WORKDIR /overlay_ws/src
COPY overlay.repos .
RUN vcs import < overlay.repos

WORKDIR /overlay_ws

RUN source /bot_mini_ws/install/setup.bash \
 && apt-get update \
 && apt-get install -y ros-humble-image-transport \
 && colcon build --symlink-install \
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
RUN source /bot_mini_ws/install/setup.bash \
 && colcon build --symlink-install \
 && apt-get update \
 && apt-get install -y --no-install-recommends ros-${ROS_DISTRO}-rviz2 \
 && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y \
 && rm -rf /var/lib/apt/lists/*

# Set up the entrypoint
ENTRYPOINT [ "/entrypoint.sh" ]

LABEL org.opencontainers.image.source=https://github.com/bjoernellens1/cps_loki

#############################################
# OliveTin Build                            #
#############################################
# FROM registry.fedoraproject.org/fedora:39 AS olivetin

# LABEL org.opencontainers.image.source https://ghcr.io/bjoernellens1/cps_bot_mini_ws/olivetin
# LABEL org.opencontainers.image.title=OliveTin

# # Making sure there is newest docker version installed
# RUN dnf -y install dnf-plugins-core \
#     && dnf -y install 'dnf-command(config-manager)' \
#     && dnf config-manager --add-repo https://download.docker.com/linux/fedora/docker-ce.repo \
#     && dnf -y install docker-ce-cli docker-compose-plugin --setopt=install_weak_deps=False

# # Install OliveTin and dependencies
# RUN mkdir -p /config /var/www/olivetin \
#     && dnf install -y --nodocs --noplugins --setopt=keepcache=0 --setopt=install_weak_deps=0 \
# 		iputils \
# 		openssh-clients \
# 		shadow-utils \
#         OliveTin \
# 	&& dnf clean all

# RUN useradd --system --create-home olivetin -u 1000

# EXPOSE 1337/tcp

# VOLUME /config

# USER olivetin

# ENTRYPOINT [ "/usr/bin/OliveTin" ]

FROM jamesread/olivetin AS olivetin

USER root

RUN curl https://download.docker.com/linux/fedora/36/x86_64/stable/Packages/docker-compose-plugin-2.18.1-1.fc36.x86_64.rpm --output docker-compose-plugin.rpm

RUN microdnf -y install dnf \
    && dnf install -y ./docker-compose-plugin.rpm \
    && dnf clean all

USER olivetin