***
# ros2 pkg命令

```
  create       Create a new ROS 2 package
  executables  Output a list of package specific executables
  list         Output a list of available packages
  prefix       Output the prefix path of a package
  xml          Output the XML of the package manifest or a specific tag
```
例子：
ros2 pkg executables列出包内所有可执行节点
```
ros2 pkg executables tf2_ros
```
return:

```
tf2_ros buffer_server
tf2_ros static_transform_publisher
tf2_ros tf2_echo
tf2_ros tf2_monitor
```
*******
## ros2 pkg create
Navigate into ros2_ws/src and create a new package:

```ros2 pkg create --build-type ament_python --license Apache-2.0 <package_name>```




### use the optional argument --node-name
```ros2 pkg create --build-type ament_python --license Apache-2.0 --node-name my_node my_package --license Apache-2.0```


### The --dependencies argument will automatically add the necessary dependency lines to package.xml

```ros2 pkg create --build-type ament_python --license Apache-2.0 py_srvcli --dependencies rclpy example_interfaces```

*************
# Build and run
*************
### in the root of your workspace (ros2_ws) to check for missing dependencies before building:

```rosdep install -i --from-path src --rosdistro humble -y```

### Still in the root of your workspace, ros2_ws, build your new package:

```colcon build --packages-select py_pubsub```



