# EUSART - Enhanced Universal Synchronous Asynchronous Receiver Transmitter.

## Code.

- EUSART TX - Display Strings.

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
; -preset_vec=0000h, -pcinit=0005h, -pstringtext=3FC0h.
; Instruction ~500ns @8MHz.

; TBOT - EUSART TX/RX.
; Echo RX Character.

; GPR BANK0.
PSECT   cstackBANK0,class=BANK0,space=1,delta=1
u8EusartRX:   DS  1

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
#define	LED_DEBUG   0x6
; ASCII Characters.
#define	ASCII_CR    0xD

; Reset Vector.
PSECT   reset_vec,class=CODE,space=0,delta=2
resetVector:
    GOTO    main

; Main.
PSECT   cinit,class=CODE,space=0,delta=2
main:
    ; MCU Initialization.
    ; Internal Oscillator Settings.
    MOVLB   BANK1
    MOVLW   0b00000110
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
    MOVLW   0b01000000
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
    MOVLW   0b00000000
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
    ; PPS Settings.
    ; PPS Write Enable.
    MOVLB   BANK28
    MOVLW   0x55
    MOVWF   PPSLOCK
    MOVLW   0xAA
    MOVWF   PPSLOCK
    BCF	    PPSLOCK, 0x0
    ; PPS Inputs.
    ; RB7 - EUSART.URX.
    movlw   0x0F
    movwf   RXPPS
    ; PPS Outputs.
    MOVLB   BANK29
    ; RB6 - EUSART.UTX.
    MOVLW   0x24
    MOVWF   RB6PPS
    ; PPS Write Disable.
    MOVLB   BANK28
    MOVLW   0x55
    MOVWF   PPSLOCK
    MOVLW   0xAA
    MOVWF   PPSLOCK
    BSF	    PPSLOCK, 0x0

    ; EUSART Settings.
    ; 9600,8,N,1.
    MOVLB   BANK3
    CLRF    RC1REG
    CLRF    TX1REG
    MOVLW   12
    MOVWF   SP1BRGL
    CLRF    SP1BRGH
    MOVLW   0x10
    MOVWF   RC1STA
    MOVLW   0x20
    MOVWF   TX1STA
    MOVLW   0x00
    MOVWF   BAUD1CON
    ; EUSART Enable.
    BSF	    SPEN

    ; EUSART Display Strings.
    CALL    _writeStringTRONIX
    CALL    _writeStringURL
    CALL    _writeStringTBOT
    CALL    _writeStringREADY

loop:
    ; EUSART Echo Character.
    CALL    _eusartRX
    CALL    _eusartTX

    ; Carriage Return ?
    MOVLB   BANK0
    MOVLW   ASCII_CR
    XORWF   u8EusartRX, W
    BTFSS   STATUS, Z
    BRA	    loop

    ; EUSART String READY.
    CALL    _writeStringREADY

    BRA	    loop

; Functions.
_eusartRX:
    MOVLB   BANK0
    BTFSS   RCIF
    BRA	    $-1
    MOVLB   BANK3
    BTFSS   OERR
    BRA	    $+3
    BCF	    CREN
    BSF	    CREN
    MOVF    RC1REG, W
    MOVLB   BANK0
    MOVWF   u8EusartRX
    RETURN

_eusartTX:
    MOVLB   BANK3
    BTFSS   TRMT
    BRA	    $-1
    MOVWF   TX1REG
    RETURN

_eusartTXString:
    MOVIW   FSR1++
    ANDLW   0xFF
    BTFSC   STATUS, Z
    RETURN
    CALL    _eusartTX
    BRA	    $-5

_writeStringREADY:
    MOVLW   HIGH stringREADY + 0x80
    MOVWF   FSR1H
    MOVLW   LOW stringREADY
    MOVWF   FSR1L
    CALL    _eusartTXString
    RETURN

_writeStringTBOT:
    MOVLW   HIGH stringTBOT + 0x80
    MOVWF   FSR1H
    MOVLW   LOW stringTBOT
    MOVWF   FSR1L
    CALL    _eusartTXString
    RETURN

_writeStringTRONIX:
    MOVLW   HIGH stringTRONIX + 0x80
    MOVWF   FSR1H
    MOVLW   LOW stringTRONIX
    MOVWF   FSR1L
    CALL    _eusartTXString
    RETURN

_writeStringURL:
    MOVLW   HIGH stringURL + 0x80
    MOVWF   FSR1H
    MOVLW   LOW stringURL
    MOVWF   FSR1L
    CALL    _eusartTXString
    RETURN

; FPM Strings.
PSECT   stringtext,class=STRCODE,space=0,delta=2
stringREADY:
    DB  0xD, 0xA, 0xD, 0xA, 'R','e','a','d','y','>',' ', 0x0

stringTBOT:
    DB  0xD, 0xA, 'T','B','O','T', 0x0

stringTRONIX:
    DB  0xD, 0xA, 0xD, 0xA, 'T','r','o','n','i','x',' ','I','/','O','.', 0x0

stringURL:
    DB  0xD, 0xA, 'w','w','w','.','t','r','o','n','i','x','.','c','o','m', 0x0

    END resetVector
```

## Oscilloscope.

- MCU.RB6.EUSART.TX -> Oscilloscope Probe A

<p align="center">
<img alt="MCU.RB6.EUSART.TX" src="https://github.com/tronixio/robot-tbot/blob/main/Code/extras/TEK00001.png">
</p>

## Terminal.

- EUSART TX - Display Strings.

<p align="center">
<img alt="EUSART TX" src="https://github.com/tronixio/robot-tbot/blob/main/Code/extras/eusart-0.png">
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
