************
# Navigate into ros2_ws/src and create a new package:
************
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



