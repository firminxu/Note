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

# start docker

```
sudo systemctl start docker
```

# To list volumes. type
```
docker volume ls

```
# To create a new volume type
```
docker volume create myvol
```
# To remove a volume type

```
docker volume rm myvol
```
# To mount a volume,type

```
docker run --mount source=myvol, target=/app
```

# To pull an image, type:

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
# 开始运行特定容器，不创建新的容器

```
docker start -i <container ID>
```
******
# 在raspi OS运行docker，载入image后创建新的容器


```
docker run -it <respotory><tag>

docker run -it ros:humble-perception-jammy 
```