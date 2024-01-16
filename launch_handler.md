https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Using-Event-Handlers.html

```
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            LogInfo, RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)
from launch.events import Shutdown
from launch.substitutions import (EnvironmentVariable, FindExecutable,
                                LaunchConfiguration, LocalSubstitution,
                                PythonExpression)


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
            FindExecutable(name='ros2'),
            ' service call ',
            turtlesim_ns,
            '/spawn ',
            'turtlesim/srv/Spawn ',
            '"{x: 2, y: 2, theta: 0.2}"'
        ]],
        shell=True
    )
    change_background_r = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' param set ',
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
            FindExecutable(name='ros2'),
            ' param set ',
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
        RegisterEventHandler(
            OnProcessStart(
                target_action=turtlesim_node,
                on_start=[
                    LogInfo(msg='Turtlesim started, spawning turtle'),
                    spawn_turtle
                ]
            )
        ),
        RegisterEventHandler(
            OnProcessIO(
                target_action=spawn_turtle,
                on_stdout=lambda event: LogInfo(
                    msg='Spawn request says "{}"'.format(
                        event.text.decode().strip())
                )
            )
        ),
        RegisterEventHandler(
            OnExecutionComplete(
                target_action=spawn_turtle,
                on_completion=[
                    LogInfo(msg='Spawn finished'),
                    change_background_r,
                    TimerAction(
                        period=2.0,
                        actions=[change_background_r_conditioned],
                    )
                ]
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=turtlesim_node,
                on_exit=[
                    LogInfo(msg=(EnvironmentVariable(name='USER'),
                            ' closed the turtlesim window')),
                    EmitEvent(event=Shutdown(
                        reason='Window closed'))
                ]
            )
        ),
        RegisterEventHandler(
            OnShutdown(
                on_shutdown=[LogInfo(
                    msg=['Launch was asked to shutdown: ',
                        LocalSubstitution('event.reason')]
                )]
            )
        ),
    ])
```



OnProcessStart、OnProcessIO、OnExecutionComplete、OnProcessExit 和 OnShutdown 事件的 RegisterEventHandler 操作在启动说明中定义。

OnProcessStart 事件处理程序用于注册一个回调函数，该函数在 turtlesim 节点启动时执行。它将一条消息记录到控制台，并在 turtlesim 节点启动时执行spawn_turtle操作。

```
RegisterEventHandler(
    OnProcessStart(
        target_action=turtlesim_node,
        on_start=[
            LogInfo(msg='Turtlesim started, spawning turtle'),
            spawn_turtle
        ]
    )
),
```

OnProcessIO 事件处理程序用于注册一个回调函数，该函数在spawn_turtle操作写入其标准输出时执行。它记录生成请求的结果。

```
RegisterEventHandler(
    OnProcessIO(
        target_action=spawn_turtle,
        on_stdout=lambda event: LogInfo(
            msg='Spawn request says "{}"'.format(
                event.text.decode().strip())
        )
    )
),
```

OnExecutionComplete 事件处理程序用于注册在spawn_turtle操作完成时执行的回调函数。它将消息记录到控制台，并在生成操作完成时执行change_background_r和change_background_r_conditioned操作。

```
RegisterEventHandler(
    OnExecutionComplete(
        target_action=spawn_turtle,
        on_completion=[
            LogInfo(msg='Spawn finished'),
            change_background_r,
            TimerAction(
                period=2.0,
                actions=[change_background_r_conditioned],
            )
        ]
    )
),
```

OnProcessExit 事件处理程序用于注册一个回调函数，该函数在 turtlesim 节点退出时执行。它将消息记录到控制台，并执行 EmitEvent 操作以在 turtlesim 节点退出时发出 Shutdown 事件。这意味着当 turtlesim 窗口关闭时，启动过程将关闭。

```
RegisterEventHandler(
    OnProcessExit(
        target_action=turtlesim_node,
        on_exit=[
            LogInfo(msg=(EnvironmentVariable(name='USER'),
                    ' closed the turtlesim window')),
            EmitEvent(event=Shutdown(
                reason='Window closed'))
        ]
    )
),
```

最后，OnShutdown 事件处理程序用于注册一个回调函数，该函数在要求启动文件关闭时执行。它会向控制台记录一条消息，说明为什么要求关闭启动文件。它记录带有关闭原因的消息，例如关闭 turtlesim 窗口或用户发出的 ctrl-c 信号。

```
RegisterEventHandler(
    OnShutdown(
        on_shutdown=[LogInfo(
            msg=['Launch was asked to shutdown: ',
                LocalSubstitution('event.reason')]
        )]
    )
),
```