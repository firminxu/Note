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
# node 命令
**********************
```
ros2 node list 

ros2 node info /XXXX
```

**********************
# topic 命令
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

###  publish data to a topic directly from the commane line using:
```ros2 topic pub <topic_name> <msg_type> '<args>'```

example:
```
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"

ros2 topic pub --rate 1 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"


```


**********************
# interface 命令
**********************
## interface show

### You can call services from the command line, but first you need to know the structure of the input arguments.
data type:

```ros2 interface show <type_name>```

servrice:

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

###  interface show example1: show detail of data(message)
```ros2 interface show std_msgs/msg/String ```


### interface show example2(service):
```
ros2 interface show example_interfaces/srv/AddTwoInts
```
return
```
int64 a
int64 b
---
int64 sum
```

### interface show example3(action):

```
ros2 interface show turtlesim/action/RotateAbsolute 
```
return

```
# The desired heading in radians
float32 theta
---
# The angular displacement in radians to the starting position
float32 delta
---
# The remaining rotation in radians
float32 remaining

```

## interface proto 
输出一个interface 原型，使用时在原型上加上大括号,中间加逗号，即变成一个字典

```
ros2 interface proto turtlesim/srv/Spawn
```
return:

```
"x: 0.0
y: 0.0
theta: 0.0
name: ''
"

```

## 其它
```
  list      List all interface types available
  package   Output a list of available interface types  
            within one package
  packages  Output a list of packages that provide interfaces

```

***
# service 命令

```
ros2 service call
ros2 service find
ros2 service info <service_name>
--include-hidden-services
ros2 service list


```
### To find out the type of a service

```
ros2 service type <service_name>
```
example:

```
ros2 service type /spawn
```
return

```
turtlesim/srv/Spawn
```
### to see the types of all the active services at the same time
```ros2 service list -t```

### If you want to find all the services of a specific type, you can use the command:

```ros2 service find <type_name>```
### you can call a service using:

```ros2 service call <service_name> <service_type> <arguments>```

example1:

```
ros2 service call /clear std_srvs/srv/Empty

ros2 service call /spawn turtlesim/srv/Spawn "{x: 2, y: 2, theta: 0.2, name: ''}"

ros2 service call /turtle1/set_pen turtlesim/srv/SetPen "{r: 100, g: 0, b: 0, width: 1, 'off': 0}"


ros2 service call /kill turtlesim/srv/Kill "{name: 'turtle2'}"

```
example2:
```
ros2 service call /add_two_ints  example_interfaces/srv/AddTwoInts "{'a': 2,'b': 4}"

```
example3(-r N, --rate N  Repeat the call at a specific rate in Hz
 ):

```
ros2 service call -r 0.5 /spawn turtlesim/srv/Spawn "{x: 5,y: 5,theta: 0}"
```
***
# action 命令

```
ros2 action info
ros2 action list
ros2 action send_goal
```
例子：--feedback 持续输出feedback
```
ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute {'theta: -1.57'} --feedback
```
***
# ros2 bag 命令

  convert  Given an input bag, write out a new bag with 
            different settings
  info     Print information about a bag to the screen
  list     Print information about available plugins to the screen
  play     Play back ROS data from a bag
  record   Record ROS data to a bag
  reindex  Reconstruct metadata file for a bag

例子：

```
ros2 bag record /turtle1/pose -o velocities


```

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
# param 命令
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

*****
# tf2 命令
*****
查看frame：
```ros2 run tf2_tools view_frames```
查看frame相对位置
```ros2 run tf2_ros tf2_echo [source_frame] [target_frame]```




