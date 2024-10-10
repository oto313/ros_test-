#!/usr/bin/env bash

for repo in src/moveit/moveit2/moveit2.repos $(f="moveit2/moveit2_$ROS_DISTRO.repos"; test -r $f && echo $f); do vcs import src/moveit < "$repo"; done

sudo rosdep update
sudo rosdep install --from-paths src --ignore-src -y && sudo chown -R $(whoami) /home/ws/