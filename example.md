**************
# Get date format from a topic
**************
input:
```
ros2 topic list 
```
return:

```
/parameter_events
/rosout
/turtle1/cmd_vel
/turtle1/color_sensor
/turtle1/pose
/turtle2/cmd_vel
/turtle2/color_sensor
/turtle2/pose

```
input:
```
ros2 topic type /turtle1/cmd_vel 
```
return:

```geometry_msgs/msg/Twist```

input:

```ros2 interface show geometry_msgs/msg/Twist```

return:

```
# This expresses velocity in free space broken into its linear and angular parts.

Vector3  linear
	float64 x
	float64 y
	float64 z
Vector3  angular
	float64 x
	float64 y
	float64 z

```
*******************************
# an example of service working
*******************************
1. input
```
ros2 service list 
```
return

```
/clear
/kill
/reset
/spawn
/turtle1/set_pen
/turtle1/teleport_absolute
/turtle1/teleport_relative
/turtlesim/describe_parameters
/turtlesim/get_parameter_types
/turtlesim/get_parameters
/turtlesim/list_parameters
/turtlesim/set_parameters
/turtlesim/set_parameters_atomically
```
2. input
```
ros2 service type /turtle1/set_pen
```
return
```

turtlesim/srv/SetPen
```

3. input
```
ros2 interface show turtlesim/srv/SetPen
```
return

```

uint8 r
uint8 g
uint8 b
uint8 width
uint8 off
---
```
4. input
```
ros2 service call /turtle1/set_pen turtlesim/srv/SetPen  "{'r': 255,'g': 0,'b': 0,'width': 3,'off': 0}"
```
return:

```
requester: making request: turtlesim.srv.SetPen_Request(r=255, g=0, b=0, width=3, off=0)

response:
turtlesim.srv.SetPen_Response()
```


**********************
# create workspace
**********************
```
mkdir ros2_ws
cd ros2_ws
mkdir src

```
### in ros2_ws folder 
```
colcon build
```
(if there is err when do colcon build, downgrade setuptools to 58.2.0)
pip3 install setuptools==58.2.0

#after that you can see folder "install", "build" and "log"
#after adding "source ~/ros2_ws/install/setup.bash" to ~/.bashrc, you don't need to source setup.bash everytime during working on ros2_ws project

### in folder src, create package "my_robot_controller" with rclpy
```
ros2 pkg create my_robot_controller --build-type ament_python --dependencies rclpy
```

### after that you can see new folder ""my_robot_controller" folder", "resource" and "test" in "my_robot_controller" folder 

### in "my_robot_controller" folder(in "my_robot_controller" folder)


*********************
# colcon build after create a new file in the package
********************
1. change below in setup.py
```
entry_points={
        'console_scripts': []
}
```
*****
example:
```
entry_points={
        'console_scripts': [
            'test_node = my_robot_controller.my_first_node:main'
        ],
    },
```
explaination: node name, package name, file name, main
*****

2. compile with below command
```
colcon build --symlink-install 
```
3. source or open a new cmd window sources automaticlyy
```
source ~/.bashrc
```