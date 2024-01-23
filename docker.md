# 安装docker

more details see:

https://www.kevsrobots.com/learn/learn_ros/04_docker_install.html

```
curl -fsSL https://get.docker.com -o get-docker.sh
chmod +x get-docker.sh 
```

# 拉取portainer

```
sudo docker pull portainer/portainer-ce:linux-arm

```

# 设置portainer gui

```
sudo docker run -d -p 9000:9000 --name=portainer --restart=always -v /var/run/docker.sock:/var/run/docker.sock -v portainer_data:/data portainer/portainer-ce:linux-arm



```
# 在浏览器打开portainer

```
<ip>:9000
```

# 在系统启动中加入docker

```
sudo systemctl start docker
```



# 拉取image:

```
docker pull docker/getting-started
```
# then type
```
docker image ls
```
# 清理未使用的images:
```
docker image prune
```

# remove a specific image

```
docker images rm imagename
```

# list all the container

```
docker ps -a
```

# docker remove the tag, but not the image.

```
docker rmi <repository>:<tag>
```


# Stop a running container, from within the container, To disconnect from a running container type CTRL+p, CTRL+q

```
exit
```

# list all currently running containers

```
docker ps
```

# list all the images

```
docker images
```

# run image in the container

```
docker run <respository><tag>
```
*****
# run container in background and print the container ID

```
docker run -d <respository><tag>
```


# stop certain container

```
docker stop <container ID>
```

# publish a container's port to the host, -p

```
docker run -d -p 8000:80 <respository><tag>
```
# 容器操作
*****
# 在raspi OS运行docker，并在运行后删除容器


```
docker run -it --rm <respotory><tag>

docker run -it --rm ros:humble-perception-jammy 
```

-i：以交互模式运行容器，通常与-t同时使用

-d：后台运行容器，并返回容器ID，也即启动守护式容器

-t：为容器重新分配一个伪输入终端，通常与-i同时使用

--rm: 让容器在退出时,自动清除挂载的卷,以便清除数据

******
## 开始运行特定容器，不创建新的容器

```
docker start -i <container ID>


```

## 删除特定容器
```docker container rm <container ID>```

******
## 在raspi OS运行docker，载入image后创建新的容器


```
docker run -it <respotory><tag>

docker run -it ros:humble-perception-jammy 
```
# volume操作

## 列出volumes
```
docker volume ls

```
## 创建新volume
```
docker volume create myvol
```
## 移除volume

```
docker volume rm myvol
```
## The following example mounts the volume myvol into /app/ in the new container named "devtest".

```


docker run -d --name devtest --mount source=myvol,target=/app/ [image]

# example:
docker run --mount source=myvol,target=/app/ nginx:latest

```
或者

```
docker run -v myvol:/app/data [image] 

```

******
## 从docker复制文件到docker volume

```docker cp <source_container>:<path/to/file> my_volume:/destination/folder```

这里需要将 <source_container> 替换为源容器的ID，<path/to/file> 替换为要复制的文件路径，/destination/folder 替换为目标文件夹路径。如果不存在目标文件夹，则会自动创建。

等待复制过程完成。

现在，文件就被成功地复制到了Docker Volume中。

注意事项：

若要从主机系统复制文件到Docker Volume，只需将 source_container 参数更改为本地文件路径。

若要从Docker Volume复制文件到主机系统，只需将 /destination/folder 参数更改为本地文件路径。

## 从本地复制文件到docker volume

```docker cp /path/to/localfile <container_name or container_id>:<destination_volume>```

*****


## 把docker volume挂载到己有的container




# Bind Mounts 新容器

可以使用-v参数将主机上的目录与容器内部的路径进行关联。示例命令如下所示：

```
docker run -it --name mycontainer -v /host/path:/container/path image_name

docker run -it --name xxxx -v /home/pi/data/:/path/to/folder ros:humble-perception-jammy 

例子：

docker run -it --name robot -v /home/pi/data/:/usr/data ros:humble-perception-jammy

```

# Bind Mounts 已有容器, 不支持树莓派

```
docker container update --mount type=bind,source=<host_folder_path>,target=<container_folder_path> <container_name>
docker container update --mount type=bind,source=/home/pi/data,target=/usr/data crazy_dirac

docker container update --mount-add type=bind,source=/path/on/host,target=/path/in/container CONTAINER_NAME

docker container update --mount -add type=bind,source=/home/pi/data,target=/usr/data crazy_dirac

docker container update -v type=bind,source=/home/pi/data,target=/usr/data crazy_dirac
```

# 文件复制操作
## 复制主机文件到docker,（已经验证）

```
docker cp <源路径> <目标容器名称或ID>:<目标路径>
```
其中， <源路径> 表示主机上的文件路径； <目标容器名称或ID> 表示要复制到的 Docker 容器的名称或 ID； <目标路径> 表示在容器内部存放该文件的路径。

举个例子，假设我们有一个名为 mycontainer 的容器，并想将 /path/to/hostfile.txt 这个文件从主机复制到容器中的 /app/ 目录下，则可以运行以下命令：

```
docker cp /path/to/hostfile.txt mycontainer:/app/

#example：
docker cp /home/pi/test.txt 88e1ed11b40e:/usr/share

```
*****
## 复制docker里的文件到本地

```docker cp <container_id>:<source> <destination>```

