https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Using-ROS2-Launch-For-Large-Projects.html

# 简介
机器人上的大型应用通常涉及多个互连的节点，每个节点可以有许多参数。 海龟模拟器中模拟多只海龟就是一个很好的例子。 海龟模拟由多个海龟节点、世界配置以及 TF 广播器和监听器节点组成。 在所有节点之间，存在大量影响这些节点的行为和外观的 ROS 参数。 ROS 2 launch文件允许我们在一个地方启动所有节点并设置相应的参数。 在教程结束时，您将在 launch_tutorial 包中构建 launch_turtlesim.launch.py launch文件。 该launch文件将调出不同的节点，负责模拟两个turtlesim 模拟、启动 TF 广播器和监听器、加载参数以及启动 RViz 配置。 在本教程中，我们将详细介绍此launch文件以及使用的所有相关功能。

# 编写launch文件
## 1.顶层组织

编写launch文件过程的目标之一应该是使它们尽可能可重用。 这可以通过将相关节点和配置聚集到单独的launch文件中来完成。 之后，可以编写专用于特定配置的顶级launch文件。 这将允许在相同的机器人之间移动而无需更改launch文件。 即使是从真实机器人转移到模拟机器人等改变也只需进行一些更改即可完成。 我们现在将回顾使这成为可能的顶级启动文件结构。 首先，我们将创建一个启动文件，该文件将调用单独的启动文件。 为此，我们在以下位置创建一个 launch_turtlesim.launch.py 文件：

```
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
   turtlesim_world_1 = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('launch_tutorial'), 'launch'),
         '/turtlesim_world_1.launch.py'])
      )
   turtlesim_world_2 = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('launch_tutorial'), 'launch'),
         '/turtlesim_world_2.launch.py'])
      )
   broadcaster_listener_nodes = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('launch_tutorial'), 'launch'),
         '/broadcaster_listener.launch.py']),
      launch_arguments={'target_frame': 'carrot1'}.items(),
      )
   mimic_node = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('launch_tutorial'), 'launch'),
         '/mimic.launch.py'])
      )
   fixed_frame_node = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('launch_tutorial'), 'launch'),
         '/fixed_broadcaster.launch.py'])
      )
   rviz_node = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('launch_tutorial'), 'launch'),
         '/turtlesim_rviz.launch.py'])
      )

   return LaunchDescription([
      turtlesim_world_1,
      turtlesim_world_2,
      broadcaster_listener_nodes,
      mimic_node,
      fixed_frame_node,
      rviz_node
   ])
```
该launch文件包括一组其他launch文件。 这些包含的launch文件中的每一个都包含节点、参数，可能还包含嵌套包含，它们属于系统的一部分。 确切地说，我们启动了两个turtlesim模拟世界，TF广播器、TF监听器、模仿器、固定帧广播器和RViz节点。

*****
设计提示：顶级启动文件应该很短，包含与应用程序子组件相对应的其他文件以及经常更改的参数。
*****
通过以下方式编写launch文件可以轻松更换系统的一部分，我们稍后会看到。 然而，在某些情况下，由于性能和使用原因，某些节点或启动文件必须单独启动。

*****
设计提示：在决定应用程序需要多少个顶级启动文件时，请注意权衡。
*****
## 2.参数
### 2.1 在launch文件中设置参数
我们将首先编写一个launch文件来启动我们的第一个turtlesim 模拟。 首先，创建一个名为turtlesim_world_1.launch.py 的新文件。

return解释：相当于命令行：
```
ros2 run turtlesim turtlesim_node --ros-args --remap __node:=sim

ros2 param set /sim  background_r 0
ros2 param set /sim  background_g 84
ros2 param set /sim  background_b 122


```

```
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution

from launch_ros.actions import Node


def generate_launch_description():
   background_r_launch_arg = DeclareLaunchArgument(
      'background_r', default_value=TextSubstitution(text='0')
   )
   background_g_launch_arg = DeclareLaunchArgument(
      'background_g', default_value=TextSubstitution(text='84')
   )
   background_b_launch_arg = DeclareLaunchArgument(
      'background_b', default_value=TextSubstitution(text='122')
   )

   return LaunchDescription([
      background_r_launch_arg,
      background_g_launch_arg,
      background_b_launch_arg,
      Node(
         package='turtlesim',
         executable='turtlesim_node',
         name='sim',
         parameters=[{
            'background_r': LaunchConfiguration('background_r'),
            'background_g': LaunchConfiguration('background_g'),
            'background_b': LaunchConfiguration('background_b'),
         }]
      ),
   ])
```
该launch文件启动turtlesim_node节点，该节点使用定义并传递给节点的模拟配置参数启动turtlesim模拟。

### 2.2 从YAML文件中加载参数
在第二个启动中，我们将使用不同的配置启动第二次turtlesim 模拟。 现在创建一个turtlesim_world_2.launch.py 文件。

return解释：相当于命令行：
```
ros2 run turtlesim turtlesim_node --ros-args --remap __ns:=/turtlesim2 --remap __node:=sim
ros2 param load /turtlesim2/sim turtlesim.yaml

```
```
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
   config = os.path.join(
      get_package_share_directory('launch_tutorial'),
      'config',
      'turtlesim.yaml'
      )

   return LaunchDescription([
      Node(
         package='turtlesim',
         executable='turtlesim_node',
         namespace='turtlesim2',
         name='sim',
         parameters=[config]
      )
   ])
```
此launch文件将使用直接从 YAML 配置文件加载的参数值启动相同的turtlesim_node。 在 YAML 文件中定义实参和参数可以轻松存储和加载大量变量。 此外，还可以轻松地从当前 ros2 参数列表中导出 YAML 文件。 要了解如何执行此操作，请参阅了解参数教程。

现在让我们在包的 /config 文件夹中创建一个配置文件turtlesim.yaml，该文件将由我们的启动文件加载。

```
/turtlesim2/sim:
   ros__parameters:
      background_b: 255
      background_g: 86
      background_r: 150
```
如果我们现在启动turtlesim_world_2.launch.py launch文件，我们将以预先配置的背景颜色启动turtlesim_node。

### 2.3 在YAML文件中使用通配符
有时我们想要在多个节点中设置相同的参数。 这些节点可以具有不同的命名空间或名称，但仍然具有相同的参数。 定义显式定义命名空间和节点名称的单独 YAML 文件效率不高。 解决方案是使用通配符（其充当文本值中未知字符的替换）将参数应用于多个不同的节点。

现在，我们创建一个与turtlesim_world_2.launch.py类似的新turtlesim_world_3.launch.py文件，以包含另一个turtlesim_node节点

```
...
Node(
   package='turtlesim',
   executable='turtlesim_node',
   namespace='turtlesim3',
   name='sim',
   parameters=[config]
)
```
然而，加载相同的 YAML 文件不会影响第三个turtlesim 世界的外观。 原因是它的参数存储在另一个命名空间下，如下所示：
```
/turtlesim3/sim:
   background_b
   background_g
   background_r
```
因此，我们可以使用通配符语法，而不是为使用相同参数的同一节点创建新配置。 /** 将分配每个节点中的所有参数，尽管节点名称和命名空间存在差异。

我们现在将按以下方式更新 /config 文件夹中的turtlesim.yaml：
```
/**:
   ros__parameters:
      background_b: 255
      background_g: 86
      background_r: 150
```


现在将turtlesim_world_3.launch.py 启动描述包含在我们的主launch文件中。 在我们的启动描述中使用该配置文件会将background_b、background_g和background_r参数分配给turtlesim3/sim和turtlesim2/sim节点中的指定值。

## 3.命名空间
您可能已经注意到，我们在turtlesim_world_2.launch.py 文件中定义了turlesim 世界的命名空间。 独特的命名空间允许系统启动两个相似的节点，而不会出现节点名称或主题名称冲突。

```namespace='turtlesim2',```

但是，如果launch文件包含大量节点，则为每个节点定义命名空间可能会变得乏味。 为了解决这个问题，可以使用 PushRosNamespace 操作为每个启动文件描述定义全局命名空间。 每个嵌套节点都会自动继承该名称空间。

为此，首先，我们需要从turtlesim_world_2.launch.py 文件中删除namespace='turtlesim2' 行。 之后，我们需要更新 launch_turtlesim.launch.py 以包含以下行：
```
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace

   ...
   turtlesim_world_2 = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('launch_tutorial'), 'launch'),
         '/turtlesim_world_2.launch.py'])
      )
   turtlesim_world_2_with_namespace = GroupAction(
     actions=[
         PushRosNamespace('turtlesim2'),
         turtlesim_world_2,
      ]
   )
```
最后，我们在 return LaunchDescription 语句中将turtlesim_world_2 替换为turtlesim_world_2_with_namespace。 因此，turtlesim_world_2.launch.py 启动描述中的每个节点都将具有turtlesim2 命名空间。

# 复用节点
现在创建一个broadcaster_listener.launch.py 文件。

```
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
   return LaunchDescription([
      DeclareLaunchArgument(
         'target_frame', default_value='turtle1',
         description='Target frame name.'
      ),
      Node(
         package='turtle_tf2_py',
         executable='turtle_tf2_broadcaster',
         name='broadcaster1',
         parameters=[
            {'turtlename': 'turtle1'}
         ]
      ),
      Node(
         package='turtle_tf2_py',
         executable='turtle_tf2_broadcaster',
         name='broadcaster2',
         parameters=[
            {'turtlename': 'turtle2'}
         ]
      ),
      Node(
         package='turtle_tf2_py',
         executable='turtle_tf2_listener',
         name='listener',
         parameters=[
            {'target_frame': LaunchConfiguration('target_frame')}
         ]
      ),
   ])
```
在此文件中，我们声明了 target_frame 启动参数，默认值为turtle1。 默认值意味着launch文件可以接收参数以转发到其节点，或者如果未提供参数，它将传递默认值到其节点。

之后，我们在启动期间使用不同的名称和参数两次使用turtle_tf2_broadcaster节点。 这允许我们复制相同的节点而不会发生冲突。

我们还启动一个turtle_tf2_listener节点并设置我们在上面声明和获取的target_frame参数。
## 5. 参数覆盖
回想一下，我们在顶级启动文件中调用了broadcaster_listener.launch.py 文件。 除此之外，我们还传递了 target_frame 启动参数，如下所示：

```
broadcaster_listener_nodes = IncludeLaunchDescription(
   PythonLaunchDescriptionSource([os.path.join(
      get_package_share_directory('launch_tutorial'), 'launch'),
      '/broadcaster_listener.launch.py']),
   launch_arguments={'target_frame': 'carrot1'}.items(),
   )
```
此语法允许我们将默认目标目标框架更改为 carrot1。 如果您希望turtle2 跟随turtle1 而不是carrot1，只需删除定义launch_arguments 的行即可。 这将为 target_frame 分配默认值，即turtle1。

## 6. 重新映射

现在创建一个mimic.launch.py 文件。
```
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
   return LaunchDescription([
      Node(
         package='turtlesim',
         executable='mimic',
         name='mimic',
         remappings=[
            ('/input/pose', '/turtle2/pose'),
            ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
         ]
      )
   ])
```
该launch文件将启动模拟节点，该节点将命令一个turtlesim跟随另一个turtlesim。 该节点旨在接收主题 /input/pose 上的目标位姿。 在我们的例子中，我们想要从 /turtle2/pose 主题重新映射目标位姿。 最后，我们将 /output/cmd_vel 主题重新映射到 /turtlesim2/turtle1/cmd_vel。 这样，我们的turtlesim2模拟世界中的turtle1将跟随我们最初的turtlesim世界中的turtle2。
## 7.配置文件
现在让我们创建一个名为turtlesim_rviz.launch.py 的文件。

return 解释：相当于命令行

```

ros2 run rviz2 rviz2 -d $(ros2 pkg prefix --share turtle_tf2_py)/rviz/turtle_rviz.rviz --ros-args --remap __name:=rviz2

```
```
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
   rviz_config = os.path.join(
      get_package_share_directory('turtle_tf2_py'),
      'rviz',
      'turtle_rviz.rviz'
      )

   return LaunchDescription([
      Node(
         package='rviz2',
         executable='rviz2',
         name='rviz2',
         arguments=['-d', rviz_config]
      )
   ])
```
此lunch文件将使用turtle_tf2_py 包中定义的配置文件启动 RViz。 此 RViz 配置将设置世界框架、启用 TF 可视化并以自上而下的视图启动 RViz。

# 8 环境变量
现在让我们在包中创建最后一个名为fixed_broadcaster.launch.py 的启动文件。
```
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
   return LaunchDescription([
      DeclareLaunchArgument(
            'node_prefix',
            default_value=[EnvironmentVariable('USER'), '_'],
            description='prefix for node name'
      ),
      Node(
            package='turtle_tf2_py',
            executable='fixed_frame_tf2_broadcaster',
            name=[LaunchConfiguration('node_prefix'), 'fixed_broadcaster'],
      ),
   ])
```
此启动文件显示了在启动文件内调用环境变量的方式。 环境变量可用于定义或推送命名空间，以区分不同计算机或机器人上的节点。
# 运行launch文件
## 1 更新setup.py
打开 setup.py 并添加以下行，以便安装 launch/ 文件夹中的启动文件和 config/ 中的配置文件。 data_files 字段现在应如下所示：
```
import os
from glob import glob
from setuptools import setup
...

data_files=[
      ...
      (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*.launch.py'))),
      (os.path.join('share', package_name, 'config'),
         glob(os.path.join('config', '*.yaml'))),
   ],
```
## 2 构建并运行
要最终查看代码的结果，请构建包并使用以下命令启动顶级启动文件：

```
ros2 launch launch_tutorial launch_turtlesim.launch.py
```
您现在将看到两个turtlesim 模拟已启动。 第一个有两只乌龟，第二个有一只。 在第一个模拟中，turtle2 在世界的左下角生成。 它的目标是到达与turtle1 框架在x 轴上相距五米的carrot1 框架。

第二个中的turtlesim2/turtle1 旨在模仿turtle2 的行为。

如果您想控制turtle1，请运行teleop 节点。
```
ros2 run turtlesim turtle_teleop_key
```
结果，你会看到类似的图片：
![Alt text](turtlesim_worlds-1.png)
除此之外，RViz 应该已经开始了。 它将显示相对于世界坐标系的所有海龟坐标系，其原点位于左下角。
![Alt text](turtlesim_rviz-1.png)
# 摘要
在本教程中，您了解了使用 ROS 2 启动文件管理大型项目的各种技巧和实践。