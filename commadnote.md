**********************
# source
**********************
### set up your ros2 humble environment by sourcing the follow file
```source /opt/ros/humble/setup.bash```


### in vscode, install ROS extension

### after adding "source /opt/ros/humble/setup.bash" to ~/.bashrc, you don't need to source the setup.bash everytime

### after adding "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" to ~/.bashrc, to enable colcon completion


### .bashrc file is a script, and by sourcing it, you execute the commands placed in that file.
```source ~/.bashrc```


**********************
# run
**********************
### run demo talker
```ros2 run demo_nodes_cpp talker```


### run demo listener
```ros2 run demo_nodes_cpp listener```


### run turtlesim node
```ros2 run turtlesim turtlesim_node ```


### run turtle_teleop_key
```ros2 run turtlesim turtle_teleop_key ```


### run service
```ros2 run demo_nodes_cpp add_two_ints_server```



**********************
# node command
**********************
```
ros2 node list 

ros2 node info /XXXX
```

**********************
# topic command
**********************

### topic list
```ros2 topic list```

### return topics list with topic type

```ros2 topic list -t```


### topic info
```ros2 topic info /XXXX```



### topic echo

```ros2 topic echo /XXXX```


### rqt_graph
```rqt_graph```

### topic hz
```ros2 topic hz /turtle1/pose```

### Remapping, reassign the name of our /turtlesim node. In a new terminal,

```ros2 run turtlesim turtlesim_node --ros-args --remap __node:=my_turtle```

###  publish data to a topic directly from the command line using:
```ros2 topic pub <topic_name> <msg_type> '<args>'```

example:
```
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"

ros2 topic pub --rate 1 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"


```


**********************
# interface command
**********************
### You can call services from the command line, but first you need to know the structure of the input arguments.

```ros2 interface show <type_name>```

example:

```ros2 interface show turtlesim/srv/Spawn```

Which will return:
```
float32 x
float32 y
float32 theta
string name # Optional.  A unique name will be created and returned if this is empty
---
string name
```

###  interface show example1: show detail of data type
```ros2 interface show std_msgs/msg/String ```


### interface show example2:
```
ros2 interface show example_interfaces/srv/AddTwoInts

int64 a
int64 b
---
int64 sum
```


### interface call example1:
```
ros2 service call /add_two_ints  example_interfaces/srv/AddTwoInts "{'a': 2,'b': 4}"


ros2 interface 
list      packages  show      
package   proto  

```

**********************
# service command
**********************
```
ros2 service call
ros2 service find
--include-hidden-services
ros2 service list

```
### To find out the type of a service

```
ros2 service type <service_name>
```
### to see the types of all the active services at the same time
```ros2 service list -t```

### If you want to find all the services of a specific type, you can use the command:

```ros2 service find <type_name>```
### you can call a service using:

```ros2 service call <service_name> <service_type> <arguments>```

example:

```ros2 service call /clear std_srvs/srv/Empty```

```ros2 service call /spawn turtlesim/srv/Spawn "{x: 2, y: 2, theta: 0.2, name: ''}"```



*******************************
# an example of service working
*******************************
1. 
```
ros2 service list 

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
2. 
```
ros2 service type /turtle1/set_pen

turtlesim/srv/SetPen
```

3. 
```
ros2 interface show turtlesim/srv/SetPen

uint8 r
uint8 g
uint8 b
uint8 width
uint8 off
---
```
4. 
```
ros2 service call /turtle1/set_pen turtlesim/srv/SetPen  "{'r': 255,'g': 0,'b': 0,'width': 3,'off': 0}"

requester: making request: turtlesim.srv.SetPen_Request(r=255, g=0, b=0, width=3, off=0)

response:
turtlesim.srv.SetPen_Response()
```

**********************
# param command
**********************
```
ros2 param list

ros2 param get <node_name> <parameter_name>

ros2 param get /turtlesim background_g

ros2 param set <node_name> <parameter_name> <value>

ros2 param set /turtlesim background_r 150

ros2 param dump <node_name>  //view all of a node’s current parameter values

ros2 param dump /turtlesim 

ros2 param dump /turtlesim > turtlesim.yaml

ros2 param load <node_name> <parameter_file>

ros2 param load /turtlesim turtlesim.yaml

ros2 run <package_name> <executable_name> --ros-args --params-file <file_name>

ros2 run turtlesim turtlesim_node --ros-args --params-file turtlesim.yaml
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

2. compile all the package

```colcon build```

compile with below command python
```
colcon build --symlink-install 
```
build only the my_package package 

```colcon build --packages-select my_package```

3. source or open a new cmd window sources automaticly
```
source ~/.bashrc
```
 from inside the ros2_ws directory, run the following command to source your workspace:

 ```source install/local_setup.bash```








