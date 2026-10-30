# AGIROS loong verification environment for the GO2 seeing-eye-dog workspace.
#
# Base: the official AGIROS unitree simulation deployment image. It already
# ships colcon, agirosdep, Gazebo Classic, the nav2 / slam_toolbox stack,
# 435 agiros-loong-* packages, pytest and PyYAML; ROS_DISTRO=loong is baked
# in as an image ENV, and the default user is 'ros' (uid 1000, sudo via
# passwordless sudoers).
#
# Build:
#   docker build -t go2-agiros-loong:2606 docker/agiros-loong/
#
# Run (workspace bind-mounted at /ws, host uid 1000 == image user 'ros'):
#   docker run --rm -it --entrypoint bash \
#     -v "$PWD":/ws -w /ws \
#     -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
#     go2-agiros-loong:2606
#
# NOTE on the AGIROS apt repo (repo.agiros.org.cn, ubuntu2204lts channels):
# as of 2026-10-30 every channel (2606/2612/latest) serves dists/jammy/Release
# WITHOUT Release.gpg or InRelease, and the documented key URL
# .../<channel>/agiros.gpg returns 404 — `apt-get update` therefore fails with
# "repository is not signed". The base image already contains every package
# this workspace builds and launches with, so no apt layer is needed here.
# If you must add packages, mark the repo trusted in a throwaway container
# (security caveat: unsigned transport):
#   echo "deb [trusted=yes] http://repo.agiros.org.cn/debrepo/agiros/ubuntu2204lts/2606 jammy main" \
#     > /etc/apt/sources.list.d/agiros.list && apt-get update
FROM docker.agiros.org.cn/sim/agiros-loong-ubuntu-unitree-sim:2606
