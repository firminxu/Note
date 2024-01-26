https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Creating-Launch-Files.html

Node1:相当于命令行：
```
ros2 run turtlesim turtlesim_node --ros-args --remap __ns:=/turtlesim1 --remap __node:=sim
```
Node2:相当于命令行：
```
ros2 run turtlesim turtlesim_node --ros-args --remap __ns:=/turtlesim2 --remap __node:=sim
```
Node3:相当于命令行：

```
ros2 run turtlesim mimic  --ros-args --remap __node:=mimic --remap /input/pose:=/turtlesim1/turtle1/pose --remap /output/cmd_vel:=/turtlesim2/turtle1/cmd_vel
```

```
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            namespace='turtlesim1',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='turtlesim',
            namespace='turtlesim2',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='turtlesim',
            executable='mimic',
            name='mimic',
            remappings=[
                ('/input/pose', '/turtlesim1/turtle1/pose'),
                ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
            ]
        )
    ])
```


