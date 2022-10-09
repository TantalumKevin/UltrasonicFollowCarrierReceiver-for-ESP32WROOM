# Ultrasonic Follow Carrier Receiver for ESP32

#### 1 介绍
本项目为2021-2022学年第二学期电气工程学院2020级卓越班嵌入式系统与智能设计课程设计：超声波跟随载物平台之超声接收端。<br>
根据设计，```ESP32WROOM32D/E```为```PDM MEMS```驱动电路控制核心MCU模组，运行C语言代码，驱动传感器与外围电路。
相关PCB设计文件，请移步仓库[Github](https://github.com/TantalumKevin/UltrasonicFollowCarrierReceiver-PCB) & [Gitee](https://gitee.com/kevin_ud/ultrasonic-follow-carrier-receiver-pcb)
#### 2 MCU模组外设选用
MCU模组外设选用：```I2S-PDM mode```（读取```MEMS```数据）、```UART```（用于与上位机通信）、```RMT```（控制```WS2812```显示当前状态）；详细配置请见```./main/USFR_main.c```

#### 3 运行流程
```C
  MCU各外设初始化->
  等待串口数据->
  发送串口数据->   
  等待串口数据->   
  //这里的三步串口是为了保证数据通畅的初始化通信
  传感器数据初始化->
┌>循环读取传感器数据->
│ 指定频率的带通滤波->
│ [根据传感器原始数据计算目标相对位置->]
│ 发送串口数据-> ─┐
└────────────────┘
```

#### 本项目其他仓库传送门
| Transmitter for Arduino UNO | Transmitter PCB | Receiver for Raspberry Pi | Receiver for STM32F103 | Receiver PCB |
| ---- | ---- | ---- | ---- | ---- |
| [Github](https://github.com/TantalumKevin/UltrasonicFollowCarrierTransmitter-for-ArduinoUNO) | [Github](https://github.com/TantalumKevin/UltrasonicFollowCarrierTransmitter-PCB) | [Github](https://github.com/TantalumKevin/UltrasonicFollowCarrierReceiver-for-RaspberryPi)  | [Github](https://github.com/TantalumKevin/UltrasonicFollowCarrierReceiver-for-STM32F103) | [Github](https://github.com/TantalumKevin/UltrasonicFollowCarrierReceiver-PCB) |
| [Gitee](https://gitee.com/kevin_ud/ultrasonic-follow-carrier-transmitter-for-arduino-uno)  | [Gitee](https://gitee.com/kevin_ud/ultrasonic-follow-carrier-transmitter-pcb) | [Gitee](https://gitee.com/kevin_ud/ultrasonic-follow-carrier)  | [Gitee](https://gitee.com/kevin_ud/ultrasonic-follow-carrier-receiver-for-stm32-f103) | [Gitee](https://gitee.com/kevin_ud/ultrasonic-follow-carrier-receiver-pcb) |