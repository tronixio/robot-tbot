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
; Instruction 495ns @8MHz.

; TBOT - v0.1.

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
#define SERVO_STOP_L	218

; Reset Vector.
PSECT reset_vec,class=CODE,space=0,delta=2
resetVect:
    goto    main

; ISR Vector.
PSECT intentry,class=CODE,space=0,delta=2
interruptVector:
    goto    isr

; Main.
PSECT cinit,class=CODE,space=0,delta=2
main:
    ; MCU Initialization.
    ; Internal Oscillator Settings.
    movlb   BANK1
    movlw   0b00000110
    movwf   OSCTUNE
    movlw   0x70
    movwf   OSCCON
    btfss   HFIOFR
    bra	    $-1
    ; Ports Settings.
    ; PORT Data Register.
    movlb   BANK0
    movlw   0b00000000
    movwf   PORTA
    movlw   0b00000000
    movwf   PORTB
    movlw   0b00000000
    movwf   PORTC
    movlw   0b00000000
    movwf   PORTE
    ; TRIS Data Direction.
    movlb   BANK1
    movlw   0b00100000
    movwf   TRISA
    movlw   0b00001001
    movwf   TRISB
    movlw   0b00000000
    movwf   TRISC
    movlw   0b00000000
    movwf   TRISE
    ; LATCH Outputs.
    movlb   BANK2
    movlw   0b00000000
    movwf   LATA
    movlw   0b00000000
    movwf   LATB
    movlw   0b00000000
    movwf   LATC
    ; ANSEL Analog.
    movlb   BANK3
    movlw   0b00000000
    movwf   ANSELA
    movlw   0b00001000
    movwf   ANSELB
    movlw   0b00000000
    movwf   ANSELC
    ; WPU Weak Pull-up.
    movlb   BANK4
    movlw   0b00000000
    movwf   WPUA
    movlw   0b00000000
    movwf   WPUB
    movlw   0b00000000
    movwf   WPUC
    movlw   0b00000000
    movwf   WPUE
    ; ODCON Open-drain.
    movlb   BANK5
    movlw   0b00000000
    movwf   ODCONA
    movlw   0b00000000
    movwf   ODCONB
    movlw   0b00000000
    movwf   ODCONC
    ; SRLCON Slew Rate.
    movlb   BANK6
    movlw   0b11111111
    movwf   SLRCONA
    movlw   0b11011111
    movwf   SLRCONB
    movlw   0b11011111
    movwf   SLRCONC
    ; INLVL Input Level.
    movlb   BANK7
    movlw   0b00000000
    movwf   INLVLA
    movlw   0b00000000
    movwf   INLVLB
    movlw   0b00000000
    movwf   INLVLC
    ; HIDRVB High Drive.
    movlb   BANK8
    movlw   0b00000000
    movwf   HIDRVB
    ; PPS Settings.
    ; PPS Write Enable.
    movlb   BANK28
    movlw   0x55
    movwf   PPSLOCK
    movlw   0xAA
    movwf   PPSLOCK
    bcf	    PPSLOCK, 0x0
    ; PPS Outputs.
    movlb   BANK29
    ; RB5 - PWM11.
    movlw   0x1F
    movwf   RB5PPS
    ; RC5 - PWM6.
    movlw   0x1E
    movwf   RC5PPS
    ; PPS Write Disable.
    movlb   BANK28
    movlw   0x55
    movwf   PPSLOCK
    movlw   0xAA
    movwf   PPSLOCK
    bsf	    PPSLOCK, 0x0

    ; PWM6 Settings.
    movlb   BANK27
    clrf    PWM6PHL
    clrf    PWM6PHH
    movlw   SERVO_STOP_L
    movwf   PWM6DCL
    movlw   SERVO_STOP_H
    movwf   PWM6DCH
    movlw   SERVO_PERIOD_L
    movwf   PWM6PRL
    movlw   SERVO_PERIOD_H
    movwf   PWM6PRH
    clrf    PWM6OFL
    clrf    PWM6OFH
    clrf    PWM6TMRL
    clrf    PWM6TMRH
    movlw   0x0C
    movwf   PWM6CON
    movlw   0x00
    movwf   PWM6INTE
    movlw   0x00
    movwf   PWM6INTF
    movlw   0x20
    movwf   PWM6CLKCON
    movlw   0x00
    movwf   PWM6LDCON
    movlw   0x00
    movwf   PWM6OFCON
    bsf	    PWM6LD
    bsf	    PWM6EN

    ; PWM11 Settings.
    movlb   BANK27
    clrf    PWM11PHL
    clrf    PWM11PHH
    movlw   SERVO_STOP_L
    movwf   PWM11DCL
    movlw   SERVO_STOP_H
    movwf   PWM11DCH
    movlw   SERVO_PERIOD_L
    movwf   PWM11PRL
    movlw   SERVO_PERIOD_H
    movwf   PWM11PRH
    clrf    PWM11OFL
    clrf    PWM11OFH
    clrf    PWM11TMRL
    clrf    PWM11TMRH
    movlw   0x0C
    movwf   PWM11CON
    movlw   0x00
    movwf   PWM11INTE
    movlw   0x00
    movwf   PWM11INTF
    movlw   0x20
    movwf   PWM11CLKCON
    movlw   0x00
    movwf   PWM11LDCON
    movlw   0x00
    movwf   PWM11OFCON
    bsf	    PWM11LD
    bsf	    PWM11EN

    ; INTERRUPTS Settings.
    BSF	    INTE
    BCF	    INTF
    ; INTERRUPTS Enabled.
    BSF	    GIE

loop:
    ; Forward.
    movlb   BANK27
    movlw   20
    movwf   PWM6DCL
    movlw   5
    movwf   PWM6DCH
    movlw   165
    movwf   PWM11DCL
    movlw   6
    movwf   PWM11DCH
    movlw   0x6
    movwf   PWMLD
    movlw   10
    call    _delay

    ; Stop.
    movlb   BANK27
    movlw   SERVO_STOP_L
    movwf   PWM6DCL
    movlw   SERVO_STOP_H
    movwf   PWM6DCH
    movlw   SERVO_STOP_L
    movwf   PWM11DCL
    movlw   SERVO_STOP_H
    movwf   PWM11DCH
    movlw   0x6
    movwf   PWMLD
    movlw   10
    call    _delay

    ; Backward.
    movlb   BANK27
    movlw   165
    movwf   PWM6DCL
    movlw   6
    movwf   PWM6DCH
    movlw   20
    movwf   PWM11DCL
    movlw   5
    movwf   PWM11DCH
    movlw   0x6
    movwf   PWMLD
    movlw   10
    call    _delay

    ; Stop.
    movlb   BANK27
    movlw   SERVO_STOP_L
    movwf   PWM6DCL
    movlw   SERVO_STOP_H
    movwf   PWM6DCH
    movlw   SERVO_STOP_L
    movwf   PWM11DCL
    movlw   SERVO_STOP_H
    movwf   PWM11DCH
    movlw   0x6
    movwf   PWMLD
    movlw   10
    call    _delay

    bra	    loop


; ISR
isr:
    ; Interrupt Emergency ?
    BTFSS   INTF
    RETFIE
    BCF	    GIE
    ; PWM6/11 Disable.
    MOVLB   BANK27
    CLRF    PWMEN
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
    movlb   BANK0
    movwf   delay + 2
    movlw   255
    movwf   delay + 1
    movlw   255
    movwf   delay
    decfsz  delay, F
    bra	    $-1
    decfsz  delay + 1, F
    bra	    $-5
    decfsz  delay + 2, F
    bra	    $-7
    return


    END	    resetVect
