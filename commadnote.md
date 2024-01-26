**********************
# ROS2 常用内容
**********************
### Source ros2 humble工作环境
```source /opt/ros/humble/setup.bash```


### 在vscode里安装ROS extension

### 把"source /opt/ros/humble/setup.bash" 命令行加入 ~/.bashrc, 会自动source, 不再需要每次打开Terminal后手动source setup.bash

### 把"source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" 命令行加入 ~/.bashrc, 使能colcon completion


### .bashrc file是一个脚本，通过下面命令手动source它（或者打开一个新的Terminal，可以执行文件内的命令。
```source ~/.bashrc```

***
# run 命令

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

***
# node 命令

```
ros2 node list 

ros2 node info /XXXX
```

**********************
# topic 命令
**********************
```
Commands:
  bw     Display bandwidth used by topic
  delay  Display delay of topic from timestamp in header
  echo   Output messages from a topic
  find   Output a list of available topics of a given type
  hz     Print the average publishing rate to screen
  info   Print information about a topic
  list   Output a list of available topics
  pub    Publish a message to a topic
  type   Print a topic's type
```
## Example:
### topic list
```ros2 topic list```

### 返回 topics list with topic type
```ros2 topic list -t```

### topic info
```ros2 topic info /XXXX```

### topic echo
```ros2 topic echo /XXXX```

### rqt_graph
```rqt_graph```

### topic hz
```ros2 topic hz /turtle1/pose```

### Remapping, reassign the name of our /turtlesim node. In a new terminal
```ros2 run turtlesim turtlesim_node --ros-args --remap __node:=my_turtle```

###  publish data to a topic directly from the commane line using:
```ros2 topic pub <topic_name> <msg_type> '<args>'```

pub example:
```
# 一次：
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
# 以1Hz的频率发送：
ros2 topic pub --rate 1 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
```
***
# interface 命令

## interface show

### 您可以从命令行调用服务，但首先您需要了解输入参数的结构。
消息（Message）数据类型:

```ros2 interface show <type_name>```

servrice:

```ros2 interface show turtlesim/srv/Spawn```

返回:
```
float32 x
float32 y
float32 theta
string name # Optional.  A unique name will be created and 返回ed if this is empty
---
string name
```

###  interface show example1: show detail of data(message)
```ros2 interface show std_msgs/msg/String ```


### interface show example2(service):
```
ros2 interface show example_interfaces/srv/AddTwoInts
```
返回
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
返回

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
返回:

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
  call  Call a service
  find  Output a list of available services of a given type
  list  Output a list of available services
  type  Output a service's type

```
### 输出服务类型

```
ros2 service type <service_name>
```
example:

```
ros2 service type /spawn
```
返回

```
turtlesim/srv/Spawn
```
### 同时查看所有活动服务的类型
```ros2 service list -t```

### 如果要查找特定类型的所有服务，可以使用以下命令：

```ros2 service find <type_name>```

### you can call a service using:
```ros2 service call <service_name> <service_type> <arguments>```

例1:

```
ros2 service call /clear std_srvs/srv/Empty

ros2 service call /spawn turtlesim/srv/Spawn "{x: 2, y: 2, theta: 0.2, name: ''}"

ros2 service call /turtle1/set_pen turtlesim/srv/SetPen "{r: 100, g: 0, b: 0, width: 1, 'off': 0}"


ros2 service call /kill turtlesim/srv/Kill "{name: 'turtle2'}"

```
例2:
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
  info       Print information about an action
  list       Output a list of action names
  send_goal  Send an action goal
```
例子：--feedback 持续输出feedback
```
ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute {'theta: -1.57'} --feedback
```
***
# ros2 bag 命令

```
  convert  Given an input bag, write out a new bag with 
            different settings
  info     Print information about a bag to the screen
  list     Print information about available plugins to the screen
  play     Play back ROS data from a bag
  record   Record ROS data to a bag
  reindex  Reconstruct metadata file for a bag
```
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
  delete    Delete parameter
  describe  Show descriptive information about declared parameters
  dump      Show all of the parameters of a node in a YAML file format
  get       Get parameter
  list      Output a list of available parameters
  load      Load parameter file for a node
  set       Set parameter
```

部分示例：

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




