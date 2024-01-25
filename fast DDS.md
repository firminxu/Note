Fast DDS Discovery Server

***
# 设置发现服务器

首先启动 ID 为 0、端口 11811（默认端口）的发现服务器并侦听所有可用接口。

打开一个新终端并运行：

```fastdds discovery --server-id 0```
返回：
```
### Server is running ###
  Participant Type:   SERVER
  Server ID:          0
  Server GUID prefix: 44.53.00.5f.45.50.52.4f.53.49.4d.41
  Server Addresses:   UDPv4:[0.0.0.0]:11811
```
***
# 启动监听节点

执行监听器演示，监听主题/chatter。

在新终端中，将环境变量设置ROS_DISCOVERY_SERVER为发现服务器的位置。 （不要忘记在每个新终端中获取 ROS 2）

```export ROS_DISCOVERY_SERVER=127.0.0.1:11811```
（因为我的电脑显示的是0.0.0.0：11811，因为发现服务器的地址也要相应的修改）
启动侦听器节点。使用参数更改本教程的节点名称。--remap __node:=listener_discovery_server
```ros2 run demo_nodes_cpp listener --ros-args --remap __node:=listener_discovery_server```
这将创建一个 ROS 2 节点，该节点将自动为发现服务器创建客户端并连接到之前创建的服务器以执行发现，而不是使用多播。
***
# 启动说话者节点
打开一个新终端并ROS_DISCOVERY_SERVER像以前一样设置环境变量，以便节点启动发现客户端。
```export ROS_DISCOVERY_SERVER=127.0.0.1:11811```
（因为我的电脑显示的是0.0.0.0：11811，因为发现服务器的地址也要相应的修改）
```ros2 run demo_nodes_cpp talker --ros-args --remap __node:=talker_discovery_server```

您现在应该看到说话者发布“hello world”消息，而侦听者接收这些消息。

***
# 演示 Discovery Server 执行
到目前为止，没有证据表明该示例与标准的说话者-收听者示例的运行方式不同。为了清楚地演示这一点，请运行另一个未连接到发现服务器的节点。在新终端中运行新的侦听器（默认情况下侦听/chatter主题）并检查它是否未连接到已运行的谈话者。
```ros2 run demo_nodes_cpp listener --ros-args --remap __node:=simple_listener```
新的侦听器节点不应接收“hello world”消息。

为了最终验证一切是否运行正常，可以使用简单发现协议（默认的DDS分布式发现机制）创建一个新的talker进行发现。
```ros2 run demo_nodes_cpp talker --ros-args --remap __node:=simple_talker```
现在您应该看到simple_listener节点接收来自 的“hello world”消息simple_talker，但不接收来自 的其他消息talker_discovery_server。

***
# 重点：
## 设置发现服务器
```fastdds discovery --server-id 0```
## 启动监听节点
```export ROS_DISCOVERY_SERVER=127.0.0.1:11811```
## 启动说话者节点
```export ROS_DISCOVERY_SERVER=127.0.0.1:11811```