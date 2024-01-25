******
# --help

在所有命令想不起来的时候，用些命令打开帮助文档


************
# 扫描局域网ip

```
sudo arp-scan -l
```

******
# 查看ubuntu版本


```
more /etc/os-release
```

******
# 查看cpu版本


```lscpu```

***
# 查看系统资源使用

```top```

***
# 查看Linux版本

```uname -a```

*****

# 在命令行中使用当前绝对目录$PWD

```
docker run -it -v $PWD/source:/my_source_code my_image
```

# && 

&&（两个&符号）：用于按顺序运行多个命令，并且只有前一个命令成功执行后，才会继续执行下一个命令。如果其中任意一个命令失败或出错，后续的命令都将停止执行。

# 不用每次输入sudo的方法
```su root```

退出root用户

```exit```或者ctrl+D