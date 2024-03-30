***
# 库配置

***
# 一、TFT_eSPI安装后,设置User_Setup.h文件

## 1. 驱动：第45行
```
#define ILI9341_DRIVER 
```

## 2. 定义ESP32的TFT显示引脚：第212到217行
```
#define TFT_MISO 19
#define TFT_MOSI 23
#define TFT_SCLK 18
#define TFT_CS   15  // Chip select control pin
#define TFT_DC    2  // Data Command control pin
#define TFT_RST   4  // Reset pin (could connect to RST pin)
```
## 3.定义ESP32的触摸屏显示引脚：第230行
```
#define TOUCH_CS 21     // Chip select pin (T_CS) of touch screen
```
## 4. 定义SPI频率：第364行

```
#define SPI_FREQUENCY  40000000
```
## 5.复制lv_conf_template.h到.pio/libdeps/esp32dev目录下, 并重命名为lv_conf.h
### 5.1  第15行 ```#if 0```改为```#if 1```
### 5.2  第233行 ```#define LV_USE_LOG 0```改为```#define LV_USE_LOG 1```
### 5.2  设置字体大小为8个像素（可选）第390行 ```#define LV_FONT_MONTSERRAT_8  0```改为```#define LV_FONT_MONTSERRAT_8  1```

***
# 二、或者直接在User_Setup_Select.h中修改，直接跳过第一步的1到4步
## 1. 注释掉第24行：```#include <User_Setup.h> ```
## 2. 选择已经预设好的配置文件，第76行：```#include <User_Setups/Setup42_ILI9341_ESP32.h>  ```
## 3. 在Setup42_ILI9341_ESP32.h文件中，取消第14行的注释```#define TOUCH_CS 5 // Chip select pin (T_CS) of touch screen```,使触摸屏生效

*** 三、连接图
![alt text](image/TFT_ESP32_Connection.jpg)

***
# 旋转配置

***
## 1. 屏幕显示旋转180度方法一：
在```lv_disp_drv_register(&disp_drv)```加上如下语句;
```disp_drv.rotated = LV_DISP_ROT_180;```
***
## 2. 屏幕显示旋转180度方法二：
把```tft.setRotation(3); /* Landscape orientation, flipped */```改成```tft.setRotation(1); /* Landscape orientation, flipped */```

***
# 触摸屏坐标超范围 (lvgl 8.3.11, TFT_eSPI 2.5.43)
```
void my_touchpad_read(lv_indev_drv_t *indev_drv, lv_indev_data_t *data)
{
    uint16_t touchX, touchY;

    bool touched = tft.getTouch(&touchX, &touchY, 600);

    if (!touched)
    {
        data->state = LV_INDEV_STATE_REL; //
    }
    else
    {
        data->state = LV_INDEV_STATE_PR;

        /*Set the coordinates*/
        data->point.x = touchX; //改之前
        data->point.y = touchY; //改之前
        data->point.x = touchX * 4 / 3; //改之后
        data->point.y = touchY * 3 / 4; //改之后

        Serial.print("Data x ");
        Serial.println(touchX);

        Serial.print("Data y ");
        Serial.println(touchY);
    }
}
```