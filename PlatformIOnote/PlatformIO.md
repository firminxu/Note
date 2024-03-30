***
# 串口监视器速度设置

在platformio.ini内加上
```
monitor_speed=115200
```
***
#在PlatformIO里，函数原型要在setup()之前声明，而在Arduino IDE里则不需要提前声明，使用示例文件时经常会出现此类错误

// Use this calibration code in setup():
  uint16_t calData[5] = { 360, 3544, 263, 3530, 7 };
  tft.setTouch(calData);