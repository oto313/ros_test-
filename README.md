### Open
* Open in VS code using devcontainers
* Manually  build docker image 
  * Run ```docker build -f .devcontainer/Dockerfile -t ros  .```
  * Run ``` docker run -it --rm ros```
### Build
 * CTRL+SHIFT+B in vscode

 * or 
   ```
    bash ros_build.sh
    ```
## Run

Run in console 1
```
source ./install/setup.bash
ros2 launch aiva_executor pnp-env.launch.py   use_hw_robot:=true
```
Run in console 2. Pos argument can be 0 or 1. If 0 the robot moves to the left otherwise to the right.
```
source ./install/setup.bash
ros2 launch aiva_executor pnp.launch.py
```