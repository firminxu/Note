***
# 串口监视器速度设置

在platformio.ini内加上
```
monitor_speed=115200
```
***
# 在PlatformIO里，函数原型要在setup()之前声明，而在Arduino IDE里则不需要提前声明，使用示例文件时经常会出现此类错误

***
# PlatformIO的编译器里让ESP32C3Supermini的串口打印使能的方法
在platformio.ini中添加
```
build_flags = 
	-D ARDUINO_USB_MODE=1
	-D ARDUINO_USB_CDC_ON_BOOT=1

