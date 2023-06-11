# IOC - Interrupt-On-Change.

## Code.

- IOC - Sharp GP2Y0D21YK.

```as
; Configuration Registers.
CONFIG FOSC=INTOSC
CONFIG WDTE=OFF
CONFIG PWRTE=OFF
CONFIG MCLRE=ON
CONFIG CP=OFF
CONFIG BOREN=OFF
CONFIG CLKOUTEN=OFF
CONFIG IESO=OFF
CONFIG FCMEN=OFF
CONFIG WRT=OFF
CONFIG PPS1WAY=ON
CONFIG ZCD=OFF
CONFIG PLLEN=OFF
CONFIG STVREN=ON
CONFIG BORV=LO
CONFIG LPBOR=OFF
CONFIG LVP=ON

#include <xc.inc>
; PIC16F1778 - Compile with PIC-AS(v2.36).
; PIC16F1778 - @8MHz Internal Oscillator.
; -preset_vec=0000h, -pintentry=0004h, -pcinit=0005h.
; Instruction ~500ns @8MHz.

; TBOT - IOC - Interrupt On Change.
; Sensor Sharp GP2Y0D21YK - Rising & Falling Edge Detection.

; GPR BANK0.
PSECT cstackBANK0,class=BANK0,space=1,delta=1
u8Delay:  DS  1

; MCU Definitions.
; BANKS.
#define	BANK0   0x0
#define	BANK1   0x1
#define	BANK2   0x2
#define	BANK3   0x3
#define	BANK4   0x4
#define	BANK5   0x5
#define	BANK6   0x6
#define	BANK7   0x7
#define	BANK8   0x8
#define	BANK9   0x9
#define	BANK10  0xA
#define	BANK11  0xB
#define	BANK12  0xC
#define	BANK13  0xD
#define	BANK14  0xE
#define	BANK15  0xF
#define	BANK16  0x10
#define	BANK17  0x11
#define	BANK18  0x12
#define	BANK19  0x13
#define	BANK20  0x14
#define	BANK21  0x15
#define	BANK22  0x16
#define	BANK23  0x17
#define	BANK24  0x18
#define	BANK25  0x19
#define	BANK26  0x1A
#define	BANK27  0x1B
#define	BANK28  0x1C
#define	BANK29  0x1D
#define	BANK30  0x1E
#define	BANK31  0x1F
; SFR STATUS Bits.
#define	C	0x0
#define	Z	0x2

; User Definition.
; LED Debug.
#define	LED_DEBUG       0x6
; Sharp GP2Y0D21YK.
#define GP2Y0D21_60MS   160
#define GP2Y0D21_ENABLE 0x0
#define GP2Y0D21_OUT    0x2

; Reset Vector.
PSECT reset_vec,class=CODE,space=0,delta=2
resetVector:
    GOTO    main

; ISR Vector.
PSECT intentry,class=CODE,space=0,delta=2
interruptVector:
    GOTO    isr

; Main.
PSECT cinit,class=CODE,space=0,delta=2
main:
    ; MCU Initialization.
    ; Internal Oscillator Settings.
    MOVLB   BANK1
    MOVLW   0b00000000
    MOVWF   OSCTUNE
    MOVLW   0x70
    MOVWF   OSCCON
    BTFSS   HFIOFR
    BRA	    $-1
    ; Ports Settings.
    ; PORT Data Register.
    MOVLB   BANK0
    MOVLW   0b00000000
    MOVWF   PORTA
    MOVLW   0b00000000
    MOVWF   PORTB
    MOVLW   0b00000000
    MOVWF   PORTC
    MOVLW   0b00000000
    MOVWF   PORTE
    ; TRIS Data Direction.
    MOVLB   BANK1
    MOVLW   0b00000000
    MOVWF   TRISA
    MOVLW   0b10011001
    MOVWF   TRISB
    MOVLW   0b01000100
    MOVWF   TRISC
    MOVLW   0b00000000
    MOVWF   TRISE
    ; LATCH Outputs.
    MOVLB   BANK2
    MOVLW   0b00000000
    MOVWF   LATA
    MOVLW   0b00000000
    MOVWF   LATB
    MOVLW   0b00000000
    MOVWF   LATC
    ; ANSEL Analog.
    MOVLB   BANK3
    MOVLW   0b00000000
    MOVWF   ANSELA
    MOVLW   0b00001000
    MOVWF   ANSELB
    MOVLW   0b00000000
    MOVWF   ANSELC
    ; WPU Weak Pull-up.
    MOVLB   BANK4
    MOVLW   0b00000000
    MOVWF   WPUA
    MOVLW   0b00000000
    MOVWF   WPUB
    MOVLW   0b00000100
    MOVWF   WPUC
    MOVLW   0b00000000
    MOVWF   WPUE
    ; ODCON Open-drain.
    MOVLB   BANK5
    MOVLW   0b00000000
    MOVWF   ODCONA
    MOVLW   0b00000000
    MOVWF   ODCONB
    MOVLW   0b00000000
    MOVWF   ODCONC
    ; SRLCON Slew Rate.
    MOVLB   BANK6
    MOVLW   0b11111111
    MOVWF   SLRCONA
    MOVLW   0b11011111
    MOVWF   SLRCONB
    MOVLW   0b11011111
    MOVWF   SLRCONC
    ; INLVL Input Level.
    MOVLB   BANK7
    MOVLW   0b00000000
    MOVWF   INLVLA
    MOVLW   0b00000000
    MOVWF   INLVLB
    MOVLW   0b00000000
    MOVWF   INLVLC
    ; HIDRVB High Drive.
    MOVLB   BANK8
    MOVLW   0b00000000
    MOVWF   HIDRVB

    ; OPTION REG Settings.
    ; WPU Enabled.
    MOVLB   BANK1
    MOVLW   0b01111111
    MOVWF   OPTION_REG

    ; GP2Y0D21 Enable.
    MOVLB   BANK2
    BSF	    LATC, GP2Y0D21_ENABLE
    ; Wait ~60ms.
    MOVLW   GP2Y0D21_60MS
    CALL    _u8Delay
    ; IOC Settings.
    MOVLB   BANK7
    BSF	    IOCCP, GP2Y0D21_OUT
    BSF	    IOCCN, GP2Y0D21_OUT
    CLRF    IOCCF

    ; INTERRUPTS Settings.
    BSF	    IOCIE
    BCF	    IOCIF
    ; INTERRUPTS Enabled.
    BSF	    GIE

loop:
    BRA	    $

;  Interrupt Service Routine.
isr:
    ; Interrupt On Change ?
    BTFSS   IOCIF
    RETFIE
    MOVLB   BANK7
    BTFSS   IOCCF, GP2Y0D21_OUT
    RETFIE
    MOVLB   BANK0
    BTFSC   PORTC, GP2Y0D21_OUT
    BRA	    $+4
    MOVLB   BANK2
    BCF	    LATA, LED_DEBUG
    BRA	    $+3
    MOVLB   BANK2
    BSF	    LATA, LED_DEBUG
    MOVLB   BANK7
    BCF	    IOCCF, GP2Y0D21_OUT
    BCF	    IOCIF
    RETFIE

; Functions.
_u8Delay:
    MOVLB   BANK0
    MOVWF   u8Delay
    MOVLW   255
    DECFSZ  WREG, F
    BRA	$-1
    DECFSZ  u8Delay, F
    BRA	$-3
    RETURN

    END resetVector
```

## MPLABX Linker Configuration.

- PIC-AS Linker > Custom linker options:
  - For Configuration & PWM: `-preset_vec=0000h, -pintentry=0004h, -pcinit=0005h`

<p align="center">
<img alt="MPLABX Linker Configuration" src="https://github.com/tronixio/robot-tbot/blob/main/pics/code-mplabx-configuration-3.png">
</p>

## Notes.

- TODO : Work in progress, prototype was not good, hardware, PCB and code can be rework.
- DRAFT : Prototype OK, last check schematic, PCB & code can be modify.

## DISCLAIMER.

THIS CODE IS PROVIDED WITHOUT ANY WARRANTY OR GUARANTEES.
USERS MAY USE THIS CODE FOR DEVELOPMENT AND EXAMPLE PURPOSES ONLY.
AUTHORS ARE NOT RESPONSIBLE FOR ANY ERRORS, OMISSIONS, OR DAMAGES THAT COULD
RESULT FROM USING THIS FIRMWARE IN WHOLE OR IN PART.

---
