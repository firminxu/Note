https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Using-Substitutions.html

在setup.py中

```
import os
from glob import glob
from setuptools import setup

package_name = 'launch_tutorial'

setup(
    # Other parameters ...
    data_files=[
        # ... Other data files
        # 包含所有launch文件，只要在这个包（package)下面的launch文件夹中，并在文件名中带有“launch.”，详情见下面通配符内容
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ]
)
```
# 父launch文件
在launch文件夹中创建example_main.launch.py

```
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution


def generate_launch_description():
    colors = {
        'background_r': '200'
    }

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('launch_tutorial'),
                    'launch',
                    'example_substitutions.launch.py'
                ])
            ]),
            launch_arguments={
                'turtlesim_ns': 'turtlesim2',
                'use_provided_red': 'True',
                'new_background_r': TextSubstitution(text=str(colors['background_r']))
            }.items()
        )
    ])
```
## 路径
FindPackageShare 用于查找 launch_tutorial 包的路径。 然后使用 PathJoinSubstitution 将该包路径的路径与 example_substitutions.launch.py 文件名连接起来。example_substitutions.launch.py 将在后面创建。

```
PathJoinSubstitution([
    FindPackageShare('launch_tutorial'),
    'launch',
    'example_substitutions.launch.py'
])
```
## 参数
带有turtlesim_ns 和use_provided_red 参数的launch_arguments 字典被传递到IncludeLaunchDescription 操作。 TextSubstitution 用于使用颜色字典中的 background_r 键的值来定义 new_background_r 参数。

```
launch_arguments={
    'turtlesim_ns': 'turtlesim2',
    'use_provided_red': 'True',
    'new_background_r': TextSubstitution(text=str(colors['background_r']))
}.items()
```

# 子launch文件
在launch目录里创建example_substitutions.launch.py
```
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    turtlesim_ns = LaunchConfiguration('turtlesim_ns')
    use_provided_red = LaunchConfiguration('use_provided_red')
    new_background_r = LaunchConfiguration('new_background_r')

    turtlesim_ns_launch_arg = DeclareLaunchArgument(
        'turtlesim_ns',
        default_value='turtlesim1'
    )
    use_provided_red_launch_arg = DeclareLaunchArgument(
        'use_provided_red',
        default_value='False'
    )
    new_background_r_launch_arg = DeclareLaunchArgument(
        'new_background_r',
        default_value='200'
    )

    turtlesim_node = Node(
        package='turtlesim',
        namespace=turtlesim_ns,
        executable='turtlesim_node',
        name='sim'
    )
    spawn_turtle = ExecuteProcess(
        cmd=[[
            'ros2 service call ',
            turtlesim_ns,
            '/spawn ',
            'turtlesim/srv/Spawn ',
            '"{x: 2, y: 2, theta: 0.2}"'
        ]],
        shell=True
    )
    change_background_r = ExecuteProcess(
        cmd=[[
            'ros2 param set ',
            turtlesim_ns,
            '/sim background_r ',
            '120'
        ]],
        shell=True
    )
    change_background_r_conditioned = ExecuteProcess(
        condition=IfCondition(
            PythonExpression([
                new_background_r,
                ' == 200',
                ' and ',
                use_provided_red
            ])
        ),
        cmd=[[
            'ros2 param set ',
            turtlesim_ns,
            '/sim background_r ',
            new_background_r
        ]],
        shell=True
    )

    return LaunchDescription([
        turtlesim_ns_launch_arg,
        use_provided_red_launch_arg,
        new_background_r_launch_arg,
        turtlesim_node,
        spawn_turtle,
        change_background_r,
        TimerAction(
            period=2.0,
            actions=[change_background_r_conditioned],
        )
    ])
```
在 example_substitutions.launch.py 文件中，定义了turtlesim_ns、use_provided_red 和 new_background_r 启动配置。 它们用于将启动参数的值存储在上述变量中，并将它们传递给所需的操作。 这些 LaunchConfiguration 允许我们获取启动描述的任何部分中的启动参数的值。

DeclareLaunchArgument 用于定义可以从上述启动文件或控制台传递的启动参数。

```
turtlesim_ns = LaunchConfiguration('turtlesim_ns')
use_provided_red = LaunchConfiguration('use_provided_red')
new_background_r = LaunchConfiguration('new_background_r')

turtlesim_ns_launch_arg = DeclareLaunchArgument(
    'turtlesim_ns',
    default_value='turtlesim1'
)
use_provided_red_launch_arg = DeclareLaunchArgument(
    'use_provided_red',
    default_value='False'
)
new_background_r_launch_arg = DeclareLaunchArgument(
    'new_background_r',
    default_value='200'
)
```
turtlesim_node 节点命名空间设置为turtlesim_ns，它在LaunchConfiguration中已经定义 。
```
turtlesim_node = Node(
    package='turtlesim',
    namespace=turtlesim_ns,
    executable='turtlesim_node',
    name='sim'
)
```
然后，使用相应的 cmd 参数定义名为 spawn_turtle 的 ExecuteProcess 操作。 该命令调用turtlesim 节点的spawn 服务。

此外，LaunchConfiguration 用于获取turtlesim_ns 启动参数的值以构造命令字符串。

```
spawn_turtle = ExecuteProcess(
    cmd=[[
        'ros2 service call ',
        turtlesim_ns,
        '/spawn ',
        'turtlesim/srv/Spawn ',
        '"{x: 2, y: 2, theta: 0.2}"'
    ]],
    shell=True
)
```
相同的方法用于更改turtlesim背景的红色参数的change_background_r和change_background_r_condition操作。 不同之处在于，仅当提供的 new_background_r 参数等于 200 并且 use_provided_red 启动参数设置为 True 时，才会执行 change_background_r_condition 操作。 IfCondition 内的评估是使用 PythonExpression 完成的。

```
change_background_r = ExecuteProcess(
    cmd=[[
        'ros2 param set ',
        turtlesim_ns,
        '/sim background_r ',
        '120'
    ]],
    shell=True
)
change_background_r_conditioned = ExecuteProcess(
    condition=IfCondition(
        PythonExpression([
            new_background_r,
            ' == 200',
            ' and ',
            use_provided_red
        ])
    ),
    cmd=[[
        'ros2 param set ',
        turtlesim_ns,
        '/sim background_r ',
        new_background_r
    ]],
    shell=True
)
```
return解释：
-turtlesim_ns_launch_arg: turtlesim_ns，缺省值为turtlesim1,在父launch文件中定义为turtlesim2
-use_provided_red_launch_arg: use_provided_red，缺省值为False,在父launch文件中定义为True
-new_background_r_launch_arg: new_background_r,缺省值为200, 在父launch文件中定义为200
-turtlesim_node: 用于创建节点：package='turtlesim', namespace=turtlesim_ns, executable='turtlesim_node', name='sim'
相当于命令行：
```
ros2 run turtlesim turtlesim_node __ns:=/turtlesim2 __node:=sim

```
-spawn_turtle: 是一个完整的命令行：
```
ros2 service call turtlesim2/spawn turtlesim/srv/Spawn "{x: 2, y: 2, theta: 0.2}"
```
-change_background_r： 是一个完整的命令行:
```
ros2 param set turtlesim2/sim background_r 120
```
-imerAction: 周期两秒，执行change_background_r_conditioned

```
    return LaunchDescription([
        turtlesim_ns_launch_arg,
        use_provided_red_launch_arg,
        new_background_r_launch_arg,
        turtlesim_node,
        spawn_turtle,
        change_background_r,
        TimerAction(
            period=2.0,
            actions=[change_background_r_conditioned],
        )
    ])
```

