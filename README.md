# STM32-BLHeli_S-DShot600
STM32 project to send arming sequence and throttle values to REV35 35A 30A BLheli_S 3-6S 4 in 1 Brushless motor controler using timer channels and DMA. The ESC requires a DSHOT600 signal in order to run the motors. The information about the arming sequence of the motors and the Dshot600 frame are described in the links bellow.

BLheli_S Manual:
https://bluerobotics.com/wp-content/uploads/2018/10/BLHeli_S-manual-SiLabs-Rev16.x.pdf

Information about Dshot600 protocol:
https://blck.mn/2016/11/dshot-the-new-kid-on-the-block/

As you may have noticed the Dshot600 frame is composed by a codified 16 bits signal. The first 11 bits are the throttle, the next one is the telemetry bit (not used in this project) and the last 4 bits are the checksum (a kind of CRC if you prefer). Each bit is a PWM signal of 1,67 us and depending on its duty cycle its recognized as a logic '0' or a logic '1'. In this project a timer is used to send a PWM signal of different duty cylces to build the main frame. The DMA is used to archive this purpose, this way this method is non blocking.

This project has been checked in a STM32F407VGT06 development board but it should work for any STM32 microcontroller by adjusting some stuff (libraries, HAL functions...) but the idea is exactly the same. If your microcontroller is not the same you can just check my code and see the configuration of the timers and the code and imitate it.

In the code a timer is used to generate the signal and another timer is used to generate a 1 ms interrupt. In the interrupt handler function a triangular shaped signal is sent (the arming sequence). Once the arming sequence is ended a variable is set and the value of a variable is converted into and array of duty cycles and sent using PWN Output signal of a timer. That variable's value can be changed in the main loop or in another function. That new value will be sent in the next interruption.

The APB bus clock, timer's prescaler and the autoreload value have been used to obtain a 1,67 us PWM singnal. You can use the value you want only if the result remains in 1,67 us. To generate a Dshot150, Dshot300 or Dshot1200 signal change that value. This example is made for Timer 1 Channel 1. But if more outputs are needed just configure more timers and channels as mine and add the needed code.

Download the .rar to download the whole project. There main code is published as a previous view.

I'm sorry for the upload format but I'm new in GitHub, I'll try to improve in the next project :)
