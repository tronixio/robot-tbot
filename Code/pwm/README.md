# PWM - 16-BIT Pulse-Width Modulation Module.

## Code.

- [PWM 16-BIT - RC Servo Forward & Backward.](https://github.com/tronixio/robot-tbot/blob/main/Code/pwm/pwm.s)

## Oscilloscope.

### Stop.

- MCU.RC5.PWM.6  -> Oscilloscope Probe A
- MCU.RB5.PWM.11 -> Oscilloscope Probe B

<p align="center">
<img alt="PWM.6 - PWM.11" src="https://github.com/tronixio/robot-tbot/blob/main/Code/extras/TEK00003.png">
</p>

### Forward.

- MCU.RC5.PWM.6  -> Oscilloscope Probe A
- MCU.RB5.PWM.11 -> Oscilloscope Probe B

<p align="center">
<img alt="PWM.6 - PWM.11" src="https://github.com/tronixio/robot-tbot/blob/main/Code/extras/TEK00004.png">
</p>

### Backward.

- MCU.RC5.PWM.6  -> Oscilloscope Probe A
- MCU.RB5.PWM.11 -> Oscilloscope Probe B

<p align="center">
<img alt="PWM.6 - PWM.11" src="https://github.com/tronixio/robot-tbot/blob/main/Code/extras/TEK00005.png">
</p>

## MPLABX Linker Configuration.

- PIC-AS Linker > Custom linker options:
  - For Configuration & PWM: `-preset_vec=0000h, -pcinit=0005h`

![MPLABX Configuration](https://github.com/tronixio/robot-tbot/blob/main/Code/extras/configuration-0.png)

## Notes.

- TODO : Work in progress, prototype was not good, hardware, PCB and code can be rework.
- DRAFT : Prototype OK, last check schematic, PCB & code can be modify.

## DISCLAIMER.

THIS CODE IS PROVIDED WITHOUT ANY WARRANTY OR GUARANTEES.
USERS MAY USE THIS CODE FOR DEVELOPMENT AND EXAMPLE PURPOSES ONLY.
AUTHORS ARE NOT RESPONSIBLE FOR ANY ERRORS, OMISSIONS, OR DAMAGES THAT COULD
RESULT FROM USING THIS FIRMWARE IN WHOLE OR IN PART.

---
