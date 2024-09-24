FROM ros:jazzy

RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y python3-pip
ENV SHELL /bin/bash
ENV XDG_RUNTIME_DIR=/tmp
USER $USERNAME
RUN apt-get update && apt install -y ros-jazzy-moveit-ros-move-group ros-jazzy-moveit-kinematics ros-jazzy-moveit-planners \
     ros-jazzy-moveit-simple-controller-manager ros-jazzy-joint-state-publisher ros-jazzy-joint-state-publisher-gui \
     ros-jazzy-xacro ros-jazzy-controller-manager ros-jazzy-moveit-configs-utils ros-jazzy-moveit-ros-visualization \
     ros-jazzy-moveit-ros-warehouse ros-jazzy-moveit-setup-assistant ros-jazzy-rviz2 ros-jazzy-rviz-common ros-jazzy-rviz-default-plugins \
     ros-jazzy-joint-state-broadcaster ros-jazzy-joint-trajectory-controller && rm -rf /var/lib/apt/lists/* 

RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
CMD ["/bin/bash"]