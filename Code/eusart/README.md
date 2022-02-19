# EUSART - Enhanced Universal Synchronous Asynchronous Receiver Transmitter.

## EUSART - TX.

### Code.

- [EUSART TX - Display Welcome Strings.](https://github.com/tronixio/robot-tbot/blob/main/Code/eusart/eusart-tx.s)

### Oscilloscope.

- MCU.RB6.EUSART.TX -> Oscilloscope Probe A

<p align="center">
<img alt="MCU.RB6.EUSART.TX" src="https://github.com/tronixio/robot-tbot/blob/main/Code/extras/TEK00001.png">
</p>

### Terminal.

- EUSART TX - Display Welcome Strings.

<p align="center">
<img alt="EUSART TX" src="https://github.com/tronixio/robot-tbot/blob/main/Code/extras/eusart-0.png">
</p>

## EUSART - TX/RX.

### Code.

- [EUSART TX/RX - Echo RX Character.](https://github.com/tronixio/robot-tbot/blob/main/Code/eusart/eusart-txrx.s)

### Oscilloscope.

- MCU.RB7.EUSART.RX -> Oscilloscope Probe A
- MCU.RB6.EUSART.TX -> Oscilloscope Probe B

<p align="center">
<img alt="MCU.RB6.EUSART.TX" src="https://github.com/tronixio/robot-tbot/blob/main/Code/extras/TEK00002.png">
</p>

### Terminal.

- EUSART TX/RX - Echo RX Character.

<p align="center">
<img alt="EUSART TX/RX" src="https://github.com/tronixio/robot-tbot/blob/main/Code/extras/eusart-1.png">
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
