# 命名空间和节点名称的区别

在 ROS2 中，命名空间和节点名称具有不同但相关的用途：[1]

命名空间是一个分层字符串，附加在节点名称前面以创建完全限定的节点名称。 命名空间对于将节点组织成逻辑组并避免名称冲突非常有用。 例如，名为 /robot1/arm/joint1 的节点将位于 /robot1/arm 命名空间中。

节点名称唯一标识进程。 每个运行的节点都是一个单独的进程，用于发布和订阅主题。 节点名称是在代码或启动文件中创建 Node 对象时指定的。

命名空间和节点名称一起创建节点通信时使用的完全限定节点名称。 例如，当节点发布有关主题的消息时，它使用其完全限定名称，其中包括命名空间和节点名称。 然后，其他节点可以使用发布者的完全限定名称来订阅该主题。

总之：

命名空间 - 将节点组织成逻辑组以避免名称冲突
节点名称 - 唯一标识每个正在运行的进程/节点
完全限定名称 - <namespace>/<node name> 用于节点之间的通信
可以在启动文件中看到如何使用命名空间和节点名称的一些示例，其中为节点组指定命名空间，并且为每个节点指定一个唯一的名称。 ROS2 文档提供了有关使用命名空间和节点的更多详细信息。 如果有任何部分需要更多解释，请告诉我！ [2]

# turtlesim节点知识盲点
生成第一只乌龟时会自动生成/turtle1/cmd_vel /turtle1/color_sensor /turtle1/pose三个话题
生成第二只乌龟时会自动生成/turtle2/cmd_vel /turtle2/color_sensor /turtle2/pose三个话题

# rqt 知识盲点
```rqt```命令打开rqt界面，可以非常方便地图形化查看所有service action topic等内容


