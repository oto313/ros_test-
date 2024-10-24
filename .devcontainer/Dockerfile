FROM ros:jazzy

RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y python3-pip
ENV SHELL /bin/bash
ENV XDG_RUNTIME_DIR=/tmp
USER $USERNAME
RUN apt-get update && apt install -y \
     ros-jazzy-joint-state-publisher ros-jazzy-joint-state-publisher-gui \
     ros-jazzy-xacro ros-jazzy-controller-manager ros-jazzy-warehouse-ros \
     ros-jazzy-rviz2 ros-jazzy-rviz-common ros-jazzy-rviz-default-plugins \
     ros-jazzy-joint-state-broadcaster ros-jazzy-joint-trajectory-controller \
     ros-jazzy-moveit-msgs ros-jazzy-ros-testing ros-jazzy-gripper-controllers \
     ros-jazzy-joy ros-jazzy-launch-param-builder liburdfdom-headers-dev \
     ros-jazzy-srdfdom ros-jazzy-py-binding-tools ros-jazzy-eigen-stl-containers\
     ros-jazzy-geometric-shapes ros-jazzy-google-benchmark-vendor libfcl-dev \
     liboctomap-dev ros-jazzy-octomap-msgs ros-jazzy-osqp-vendor ros-jazzy-random-numbers \ 
     ros-jazzy-ruckig ros-jazzy-ament-cmake-google-benchmark freeglut3-dev \
     libglew-dev libomp-dev ros-jazzy-cv-bridge ros-jazzy-object-recognition-msgs \
     ros-jazzy-launch-pytest ros-jazzy-moveit-resources ros-jazzy-rmf-utils \
     ros-jazzy-ros2-control ros-jazzy-warehouse-ros-sqlite ros-jazzy-position-controllers \
     ros-jazzy-stomp ros-jazzy-ompl

RUN pip install grpcio-tools dependency-injector dynaconf dishka --break-system-packages

RUN echo "source /opt/ros/jazzy/setup.bash\n\
source /home/ws/install/setup.bash\n" >> ~/.bashrc
CMD ["/bin/bash"]