***
# rviz2闪烁

运行
```  nvidia-settings```
Click on "X Server Display Configuration". If you have multiple monitors choose the one you want to setup for stereo. Select the Resolution (do not choose "Auto"). Select the refresh rate (to the right of "Resolution"). You want a refresh rate at least 100Hz (120Hz or faster is better). Click "Apply".

***
# package重新创建问题，比如：我在工作区创建了一个名为tutorial_interfaces的包，并在其它包里引用了里面的srv文件，编译后成功运行，然后删除了。再创建同名的tutorial_interfaces包，再编译后运行时发生引用srv出错。

解决方法：要把build和install文件夹内的tutorial_interfaces目录全部删除，再编译