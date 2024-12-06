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
; PIC16F1778 - Compile with PIC-AS(v2.50).
; PIC16F1778 - @8MHz Internal Oscillator.
; Custom Linker Options:
; -preset_vec=0000h, -pintentry=0004h, -pcinit=0005h, -pstringtext=3FC0h.
; Instruction ~500ns @8MHz.

; TBOT.
; todo: ADC, EUSART Rx, Urgency (SW/LED)
; todo: ADC display batterie status dans le splah screen
;https://github.com/tronixio/robot-tbot/tree/main/Code/adc

; Pinout:
; MCU.RA6 ->  GPIO.DEBUG.LED.
; MCU.RB0 <-> GPIO.URGENCY.
; MCU.RB5 ->  PWM11.SERVO.LEFT.
; MCU.RB6 ->  EUSART.TX.MCP2221A.URx.
; MCU.RB7 <-  EUSART.RX.MCP2221A.UTx.
; MCU.RC5 ->  PWM6.SERVO.RIGHT.

; GPR BANK0.
PSECT cstackBANK0,class=BANK0,space=1,delta=1
u8BANK0:    DS  1
u8EusartRX: DS	1

; Common RAM.
PSECT cstackCOMM,class=COMMON,space=1,delta=1
u8DELAY:    DS	1
u16DELAY:   DS	1

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
; Debug LED.
#define	DEBUG_LED	LATA, 0x6
; Urgency Switch & LED.
#define URGENCY_LED	TRISB, 0x0

; RC Servo.
; Frequency 50Hz - @8MHz.
#define SERVO_PERIOD_H	0x4E
#define SERVO_PERIOD_L	0x20
; Duty Cycle.
; H6/L164 0x06A4 - 1.70ms - @8MHz.
; H5/L220 0x05DC - 1.50ms - @8MHz.
; H5/L0   0x0500 - 1.30ms - @8MHz.
#define SERVO_STOP_H	0x05
#define SERVO_STOP_L	0xDC

; Reset Vector.
PSECT reset_vec,class=CODE,space=0,delta=2
resetVector:
    GOTO    main

    ; ISR Vector.
PSECT intentry,class=CODE,space=0,delta=2
interruptVector:
    GOTO    _ISR

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
    MOVLB   BANK28
    ; RB7 - EUSART.URx.
    MOVLW   0x0F
    MOVWF   RXPPS
    ; PPS Outputs.
    MOVLB   BANK29
    ; RB5 - PWM11 Left.
    MOVLW   0x1F
    MOVWF   RB5PPS
    ; RB6 - EUSART.UTx.
    MOVLW   0x24
    MOVWF   RB6PPS
    ; RC5 - PWM6 Right.
    MOVLW   0x1E
    MOVWF   RC5PPS
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
    MOVLW   0
    MOVWF   SP1BRGH
    MOVLW   0x10
    MOVWF   RC1STA
    MOVLW   0x20
    MOVWF   TX1STA
    MOVLW   0x00
    MOVWF   BAUD1CON
    ; EUSART Enable.
    BSF	    SPEN

    ; PWM6 Settings.
    MOVLB   BANK27
    CLRF    PWM6PHL
    CLRF    PWM6PHH
    MOVLW   SERVO_STOP_L
    MOVWF   PWM6DCL
    MOVLW   SERVO_STOP_H
    MOVWF   PWM6DCH
    MOVLW   SERVO_PERIOD_L
    MOVWF   PWM6PRL
    MOVLW   SERVO_PERIOD_H
    MOVWF   PWM6PRH
    CLRF    PWM6OFL
    CLRF    PWM6OFH
    CLRF    PWM6TMRL
    CLRF    PWM6TMRH
    MOVLW   0x0C
    MOVWF   PWM6CON
    CLRF    PWM6INTE
    CLRF    PWM6INTF
    MOVLW   0x20
    MOVWF   PWM6CLKCON
    CLRF    PWM6LDCON
    CLRF    PWM6OFCON
    ; PWM6 Load & Enable.
    BSF	    PWM6LD
    BCF	    PWM6EN

    ; PWM11 Settings.
    MOVLB   BANK27
    CLRF    PWM11PHL
    CLRF    PWM11PHH
    MOVLW   SERVO_STOP_L
    MOVWF   PWM11DCL
    MOVLW   SERVO_STOP_H
    MOVWF   PWM11DCH
    MOVLW   SERVO_PERIOD_L
    MOVWF   PWM11PRL
    MOVLW   SERVO_PERIOD_H
    MOVWF   PWM11PRH
    CLRF    PWM11OFL
    CLRF    PWM11OFH
    CLRF    PWM11TMRL
    CLRF    PWM11TMRH
    MOVLW   0x0C
    MOVWF   PWM11CON
    CLRF    PWM11INTE
    CLRF    PWM11INTF
    MOVLW   0x20
    MOVWF   PWM11CLKCON
    CLRF    PWM11LDCON
    CLRF    PWM11OFCON
    ; PWM11 Load & Enable.
    BSF	    PWM11LD
    BCF	    PWM11EN

    ; Splash screen.
    CALL    writeStringTRONIX
    CALL    writeStringURL
    CALL    writeStringTBOT
    CALL    writeStringREADY

    ; INTERRUPTS Settings.
    MOVLB   BANK0
    CLRF    PIR1
    MOVLB   BANK1
    MOVLW   0b00100000
    MOVWF   PIE1
    ; INTERRUPTS Enable.
    BSF	    PEIE
    BSF	    GIE


;    bra	    $
loop:
;    movlb   0
;    movf    u8EusartRX, W
;    call    _eusartTX
;    movlw   10
;    call    _u16Delay

;   Received character.
;    call    _eusartRX
;    call    _eusartTX
;    movlw   10
;    call    _u16Delay
    
;    MOVLB   BANK27
;    MOVLW   0xa0
;    MOVWF   PWM11DCL
;    MOVLW   0x05
;    MOVWF   PWM11DCH
;    BSF	    PWM11LD
;    BSF	    PWM11EN

    BRA	    loop
    BRA	    $

; Functions.
_ISR:
    MOVLB   BANK0
    BTFSS   RCIF
    RETFIE
    CALL    _eusartRX
    RETFIE

_u8Delay:
    MOVWF   u8DELAY
    MOVLW   255
    DECFSZ  WREG, F
    BRA	    $-1
    DECFSZ  u8DELAY, F
    BRA	    $-3
    RETURN

_u16Delay:
    MOVWF   u16DELAY
    MOVLW   255
    MOVWF   u8DELAY
    MOVLW   255
    DECFSZ  WREG, F
    BRA	    $-1
    DECFSZ  u8DELAY, F
    BRA	    $-3
    DECFSZ  u16DELAY, F
    BRA	    $-5
    RETURN

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
    MOVIW   FSR0++
    ANDLW   0xFF
    BTFSC   STATUS, Z
    RETURN
    CALL    _eusartTX
    BRA	    $-5

DebugLEDOFF:
    MOVLB   BANK2
    BCF	    DEBUG_LED
    RETURN

DebugLEDON:
    MOVLB   BANK2
    BSF	    DEBUG_LED
    RETURN

writeStringREADY:
    MOVLW   LOW stringREADY
    MOVWF   FSR0L
    MOVLW   HIGH stringREADY + 0x80
    MOVWF   FSR0H
    CALL    _eusartTXString
    RETURN

writeStringTBOT:
    MOVLW   LOW stringTBOT
    MOVWF   FSR0L
    MOVLW   HIGH stringTBOT + 0x80
    MOVWF   FSR0H
    CALL    _eusartTXString
    RETURN

writeStringTRONIX:
    MOVLW   LOW stringTRONIX
    MOVWF   FSR0L
    MOVLW   HIGH stringTRONIX + 0x80
    MOVWF   FSR0H
    CALL    _eusartTXString
    RETURN

writeStringURL:
    MOVLW   LOW stringURL
    MOVWF   FSR0L
    MOVLW   HIGH stringURL + 0x80
    MOVWF   FSR0H
    CALL    _eusartTXString
    RETURN

UrgencyLEDOFF:
    MOVLB   BANK1
    BSF	    URGENCY_LED
    RETURN

UrgencyLEDON:
    MOVLB   BANK1
    BCF	    URGENCY_LED
    RETURN

; FPM Strings.
PSECT stringtext,class=STRCODE,space=0,delta=2

stringREADY:
    DB  0xD, 0xA, 'R','e','a','d','y','>',' ', 0x0

stringTBOT:
    DB  0xD, 0xA, 'T','B','O','T', ' ', '-', ' ', 'v', '0', '.', '1', 0xD, 0xA, 0x0

stringTRONIX:
    DB  0xD, 0xA, 0xD, 0xA, 'T','r','o','n','i','x',' ','I','/','O','.', 0x0

stringURL:
    DB  0xD, 0xA, 'w','w','w','.','t','r','o','n','i','x','.','c','o','m', 0x0

    END resetVector
