*****
# urdf_tutorial安装
*****

从链接
https://github.com/ros/urdf_tutorial/tree/ros2
下载zip文件到ros2_ws/src目录下，在ros2_ws目录下运行

```colcon build --packages-select urdf_tutorial ```

如果运行提示没有urdf_launch包。执行下面安装

*****
# urdf_launch 安装
*****
从链接
https://github.com/ros/urdf_launch
下载zip文件到ros2_ws/src目录下，在ros2_ws目录下运行

```colcon build --packages-select urdf_launch```

*****
# 安装其它依赖项
*****
运行以下命令，要将<ros2-distro>替换为humble
```
sudo apt install ros-<ros2-distro>-joint-state-publisher
sudo apt install ros-<ros2-distro>-joint-state-publisher-gui
sudo apt install ros-<ros2-distro>-xacro
```