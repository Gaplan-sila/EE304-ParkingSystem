# EE304-ParkingSystem
EE304 Project- Smart Parking System
Schematic :

1- Schematics were drawn on fritzing. Schematic and experimental breadboard design added.

2- The designs were added as PNG files and Fritzing files. The Fritzing program can be downloaded from the link below.

https://fritzing.org/download/ (donate version)
https://gist.github.com/RyanLua/fc2457d87641bb39754278b01a647526 (free shared version)

3- Communication interfaces have been added as labels. A single label has been added for identical systems (e.g., LED systems). These systems use the same communication interface.

4- Parts were selected for simulation as close to reality as possible. Some parts (e.g., servo motors) were selected for general purposes, and no specific model numbers were entered. The parts list can be seen below.

MTR-41898 IR928-6C Kızılötesi (Infrared) Verici Transmitter 
KSTK-10027 SG90 RC Mini Servo Motor Micro Servo Motor SG90 9g 180°
KSTK-10020 HC-SR04 Arduino Ultrasonik Mesafe Sensörü
KSTK-1011 STM32F103C6T6 Mini Geliştirme Kartı Modülü
KSTK-10004 2x16 LCD - LCD1602 Mavi Ekran + IIC/I2C Arayüz Modülü 
MTR-45749 TSOP1838 38 KHz IR Alıcı Göz - Metal

Firmware (Coding) :

1- The codes were written in embedded C language at the register level on the Keil uVision IDE compiler.The codes were written for the STM32F103C6T6A processor.

2-No high-level libraries (such as HAL or SPL) were used, low-level drivers were written by manipulating the CRL and CRH configuration registers.

3- Code for the servo control modules was written using a bit-banging approach (virtual PWM) to demonstrate direct GPIO manipulation. We were skeptical about whether using a hardware PWM timer was permissible, but PWM seems more logical for efficient use of processor resources. Nevertheless, the pins were selected to be PWM-compatible to address the possibility of future system and code updates.

4- Again, due to the GPIO coding requirements for this submission, blocking loop logic was used for distance calculation. While it worked in practice for our project, a timer might have been more logical given processor resources and measurement accuracy. However, the advantage of this approach was that we could free up a limited number of timers for other systems like servo systems or 38khz IR systems.

5- In IR communication, a timer was used as an exception at a signal frequency of 38 kHz, because the processing power required to achieve this frequency could lock up the processor.

6- A register level I2C protocol was written for communication with the LCD.


