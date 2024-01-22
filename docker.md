# install docker

more details see:

https://www.kevsrobots.com/learn/learn_ros/04_docker_install.html

```
curl -fsSL https://get.docker.com -o get-docker.sh
chmod +x get-docker.sh 
```

# pull portainer

```
sudo docker pull portainer/portainer-ce:linux-arm

```

# set portainer gui

```
sudo docker run -d -p 9000:9000 --name=portainer --restart=always -v /var/run/docker.sock:/var/run/docker.sock -v portainer_data:/data portainer/portainer-ce:linux-arm

sudo docker run -d -p 8000:8000 -p 9000:9000 --name=portainer --restart=always -v /var/run/docker.sock:/var/run/docker.sock -v portainer_data:/data portainer/portainer-ce:linux-arm


```
# get portainer in browser

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

# To pull an imamge, type:

```
docker pull docker/getting-started
```
# then type
```
docker image ls
```
# To remove unused images, type:
```
docker image prune
```

# remove a specific image

```
docker images rm imagename
```

# list all the containers including thoser not in use

```
docker container ls -all
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