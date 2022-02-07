# ROBOT - TBOT 2 Wheels.

## Code.

- [Configuration](https://github.com/tronixio/robot-tbot/blob/main/Code/configuration.s)
- [EUSART TX](https://github.com/tronixio/robot-tbot/blob/main/Code/eusart.s)
- [ADC Battery](https://github.com/tronixio/robot-tbot/blob/main/Code/adc.s)

<!-- - [TODO - RC Servo](https://github.com/tronixio/robot-tbot/blob/main/Code/rc-servo.s) -->
<!-- - [TODO - GP2Y](https://github.com/tronixio/robot-tbot/blob/main/Code/gp2y.s) -->

## Configuration.

- PIC-AS Linker > Custom linker options: -preset_vec=0000h, -pcinit=0005h, -pstringtext=3FC0h.

![MPLABX Configuration](./extras/configuration.png)


## Notes.

- TODO : Work in progress, prototype was not good, hardware, PCB and code can be rework.
- DRAFT : Prototype OK, last check schematic, PCB & code can be modify.

## DISCLAIMER.

THIS CODE IS PROVIDED WITHOUT ANY WARRANTY OR GUARANTEES.
USERS MAY USE THIS CODE FOR DEVELOPMENT AND EXAMPLE PURPOSES ONLY.
AUTHORS ARE NOT RESPONSIBLE FOR ANY ERRORS, OMISSIONS, OR DAMAGES THAT COULD
RESULT FROM USING THIS FIRMWARE IN WHOLE OR IN PART.

---
