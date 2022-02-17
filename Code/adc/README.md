# MCU Configuration.

## Code.

- [ADC/TIMER0 - Read & Display Battery Value.](https://github.com/tronixio/robot-tbot/blob/main/Code/adc/adc.s)

## Terminal.

<p align="center">
<img alt="MCU.RB6.EUSART.TX" src="https://github.com/tronixio/robot-tbot/blob/main/Code/extras/eusart-1.png">
</p>

## MPLABX Linker Configuration.

- PIC-AS Linker > Custom linker options:
  - For Configuration & PWM: `-preset_vec=0000h, -pcinit=0005h, -pstringtext=3FC0h`

![MPLABX Configuration](https://github.com/tronixio/robot-tbot/blob/main/Code/extras/configuration-1.png)

## Notes.

- TODO : Work in progress, prototype was not good, hardware, PCB and code can be rework.
- DRAFT : Prototype OK, last check schematic, PCB & code can be modify.

## DISCLAIMER.

THIS CODE IS PROVIDED WITHOUT ANY WARRANTY OR GUARANTEES.
USERS MAY USE THIS CODE FOR DEVELOPMENT AND EXAMPLE PURPOSES ONLY.
AUTHORS ARE NOT RESPONSIBLE FOR ANY ERRORS, OMISSIONS, OR DAMAGES THAT COULD
RESULT FROM USING THIS FIRMWARE IN WHOLE OR IN PART.

---
