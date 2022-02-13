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
; PIC16F1778 - Compile with PIC-AS(v2.35).
; PIC16F1778 - @8MHz Internal Oscillator.
; -preset_vec=0000h, -pintentry=0004h, -pcinit=0005h.
; Instruction ~500ns @8MHz.

; TBOT.

; PWM RC Servo & Interrupt Emergency Stop.
; Forward 1 second, Stop, Backward 1 second, Stop, Loop.

; GPR BANK0.
PSECT cstackBANK0,class=BANK0,space=1,delta=1
delay:  DS  3

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

; User Definition.
; Debug LED.
#define	LED_DEBUG	0x6
; Emergency.
#define EMERGENCY	0x0
; RC Servo.
; Frequency 50Hz - @8MHz.
#define SERVO_PERIOD_H	78
#define SERVO_PERIOD_L  30
; Duty Cycle.
; H6/L165 - 1.7ms - @8MHz.
; H5/L220 - 1.5ms - @8MHz.
; H5/L20  - 1.3ms - @8MHz.
#define SERVO_STOP_H	5
#define SERVO_STOP_L	220

; Reset Vector.
PSECT reset_vec,class=CODE,space=0,delta=2
resetVect:
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
    MOVLW   0b00100000
    MOVWF   TRISA
    MOVLW   0b00001001
    MOVWF   TRISB
    MOVLW   0b00000000
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
    ; PPS Outputs.
    MOVLB   BANK29
    ; RB5 - PWM11.
    MOVLW   0x1F
    MOVWF   RB5PPS
    ; RC5 - PWM6.
    MOVLW   0x1E
    MOVWF   RC5PPS
    ; PPS Write Disable.
    MOVLB   BANK28
    MOVLW   0x55
    MOVWF   PPSLOCK
    MOVLW   0xAA
    MOVWF   PPSLOCK
    BSF	    PPSLOCK, 0x0

    ; PWM6 Settings.
    MOVLB   BANK27
    CRLF    PWM6PHL
    CRLF    PWM6PHH
    MOVLW   SERVO_STOP_L
    MOVWF   PWM6DCL
    MOVLW   SERVO_STOP_H
    MOVWF   PWM6DCH
    MOVLW   SERVO_PERIOD_L
    MOVWF   PWM6PRL
    MOVLW   SERVO_PERIOD_H
    MOVWF   PWM6PRH
    CRLF    PWM6OFL
    CRLF    PWM6OFH
    CRLF    PWM6TMRL
    CRLF    PWM6TMRH
    MOVLW   0x0C
    MOVWF   PWM6CON
    MOVLW   0x00
    MOVWF   PWM6INTE
    MOVLW   0x00
    MOVWF   PWM6INTF
    MOVLW   0x20
    MOVWF   PWM6CLKCON
    MOVLW   0x00
    MOVWF   PWM6LDCON
    MOVLW   0x00
    MOVWF   PWM6OFCON
    BSF	    PWM6LD
    BSF	    PWM6EN

    ; PWM11 Settings.
    MOVLB   BANK27
    CRLF    PWM11PHL
    CRLF    PWM11PHH
    MOVLW   SERVO_STOP_L
    MOVWF   PWM11DCL
    MOVLW   SERVO_STOP_H
    MOVWF   PWM11DCH
    MOVLW   SERVO_PERIOD_L
    MOVWF   PWM11PRL
    MOVLW   SERVO_PERIOD_H
    MOVWF   PWM11PRH
    CRLF    PWM11OFL
    CRLF    PWM11OFH
    CRLF    PWM11TMRL
    CRLF    PWM11TMRH
    MOVLW   0x0C
    MOVWF   PWM11CON
    MOVLW   0x00
    MOVWF   PWM11INTE
    MOVLW   0x00
    MOVWF   PWM11INTF
    MOVLW   0x20
    MOVWF   PWM11CLKCON
    MOVLW   0x00
    MOVWF   PWM11LDCON
    MOVLW   0x00
    MOVWF   PWM11OFCON
    BSF	    PWM11LD
    BSF	    PWM11EN

    ; INTERRUPTS Settings.
    BSF	    INTE
    BCF	    INTF
    ; INTERRUPTS Enabled.
    BSF	    GIE

loop:
    ; Forward.
    MOVLB   BANK27
    MOVLW   20
    MOVWF   PWM6DCL
    MOVLW   5
    MOVWF   PWM6DCH
    MOVLW   165
    MOVWF   PWM11DCL
    MOVLW   6
    MOVWF   PWM11DCH
    MOVLW   0x6
    MOVWF   PWMLD
    MOVLW   10
    call    _delay

    ; Stop.
    MOVLB   BANK27
    MOVLW   SERVO_STOP_L
    MOVWF   PWM6DCL
    MOVLW   SERVO_STOP_H
    MOVWF   PWM6DCH
    MOVLW   SERVO_STOP_L
    MOVWF   PWM11DCL
    MOVLW   SERVO_STOP_H
    MOVWF   PWM11DCH
    MOVLW   0x6
    MOVWF   PWMLD
    MOVLW   10
    call    _delay

    ; Backward.
    MOVLB   BANK27
    MOVLW   165
    MOVWF   PWM6DCL
    MOVLW   6
    MOVWF   PWM6DCH
    MOVLW   20
    MOVWF   PWM11DCL
    MOVLW   5
    MOVWF   PWM11DCH
    MOVLW   0x6
    MOVWF   PWMLD
    MOVLW   10
    call    _delay

    ; Stop.
    MOVLB   BANK27
    MOVLW   SERVO_STOP_L
    MOVWF   PWM6DCL
    MOVLW   SERVO_STOP_H
    MOVWF   PWM6DCH
    MOVLW   SERVO_STOP_L
    MOVWF   PWM11DCL
    MOVLW   SERVO_STOP_H
    MOVWF   PWM11DCH
    MOVLW   0x6
    MOVWF   PWMLD
    MOVLW   10
    call    _delay

    BRA	    loop


; ISR
isr:
    ; Interrupt Emergency ?
    BTFSS   INTF
    RETFIE
    BCF	    GIE
    ; PWM6/11 Disable.
    MOVLB   BANK27
    CRLF    PWMEN
    ; LED Emergency Blink.
    MOVLB   BANK2
    BCF	    LATB, EMERGENCY
    MOVLB   BANK1
    BCF	    TRISB, EMERGENCY
    MOVLW   1
    CALL    _delay
    MOVLB   BANK1
    BSF	    TRISB, EMERGENCY
    MOVLW   5
    CALL    _delay
    BRA	    $-8

; Functions
_delay:
    MOVLB   BANK0
    MOVWF   delay + 2
    MOVLW   255
    MOVWF   delay + 1
    MOVLW   255
    MOVWF   delay
    DECFSZ  delay, F
    BRA	    $-1
    DECFSZ  delay + 1, F
    BRA	    $-5
    DECFSZ  delay + 2, F
    BRA	    $-7
    RETURN

    END	    resetVect
