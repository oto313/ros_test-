### Open
* Open in VS code using devcontainers
* Manually  build docker image 
  * Run ```docker build -f .devcontainer/Dockerfile -t ros  .```
  * Run ``` docker run -it --rm ros```
### Build
 * CTRL+SHIFT+B in vscode

 * or 
   ```
    source /opt/ros/jazzy/setup.bash && colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=Debug
    ```
## Run

Run in console 1
```
source ./install/setup.bash
ros2 launch nova2_moveit run.launch.py
```
Run in console 2. Pos argument can be 0 or 1. If 0 the robot moves to the left otherwise to the right.
```
source ./install/setup.bash
ros2 run aiva_executor aiva_executor --ros-args -p use_sim_time:=true -p pos:=<0|1>
```