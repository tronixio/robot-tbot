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

; TBOT - v0.1.
; Sensor SHARP GP2Y0D21YK.

; TODO: Rotation Rmp ?
; TODO: Better Delay Fonction
; TODO: Optimize Variables

; Pinout.
; MCU.RA6  -> LED.DEBUG - OSCILLOSCOPE.PROBE.
; MCU.RB0 <-> EMERGENCY.LED.SWITCH.
; MCU.RB3 <-  AN9.BATTERY.SENSE.
; MCU.RB5  -> PWM11.RC.SERVO.
; MCU.RB6  -> EUSART.TX.
; MCU.RB7 <-  EUSART.RX.
; MCU.RC0  -> GP2Y0D21YK.ENABLE.
; MCU.RC2 <-  GP2Y0D21YK.OUT.
; MCU.RC5  -> PWM6.RC.SERVO.

; GPR BANK0.
PSECT cstackBANK0,class=BANK0,space=1,delta=1
delay:	    DS  2
filter:	    DS	1
counter:    DS	1

; Common RAM.
PSECT cstackCOMM,class=COMMON,space=1,delta=1
TBOT:	    DS  1

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

; User Definitions.
; Battery.
#define BATTERY_FILTER	    50
#define	BATTERY_LOW	    0xB7
; Emergency.
#define EMERGENCY	    0x0
; LED Debug.
#define	LED_DEBUG	    0x6
; RC Servo.
#define	SERVO_COUNTER	    202
; Frequency 50Hz - @8MHz.
#define SERVO_PERIOD_H	    78
#define SERVO_PERIOD_L	    30
; Duty Cycle.
; H6/L165 - 1.7ms - @8MHz.
; H5/L220 - 1.5ms - @8MHz.
; H5/L20  - 1.3ms - @8MHz.
#define SERVO_STOP_H	    5
#define SERVO_STOP_L	    220
;
#define SERVO_MAX_RIGH_L    26
#define SERVO_MAX_LEFT_L    165
; Sharp GP2Y0D21YK.
#define	GP2Y0D21_60MS	    160
#define GP2Y0D21_ENABLE	    0x0
#define GP2Y0D21_OUT	    0x2
; TBOT Flags.
#define TBOT_RCSERVO	    0x0

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

    ; ADC Settings.
    ; PWM6 Duty Cycle Trigger.
    ; Left Justified, FOSC/16.
    MOVLB   BANK9
    CLRF    ADRESL
    CLRF    ADRESH
    MOVLW   0x24
    MOVWF   ADCON0
    MOVLW   0x50
    MOVWF   ADCON1
    MOVLW   0x02
    MOVWF   ADCON2
    ; ADC Enable.
    BSF	    ADON

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
    ; PWM6 Load & Enable.
    BSF	    PWM6LD
    BSF	    PWM6EN

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
    ; PWM11 Load & Enable.
    BSF	    PWM11LD
    BSF	    PWM11EN

    ; OPTION REG Settings.
    ; WPU Enabled.
    ; TIMER0 ~15Hz @8MHz.
    MOVLB   BANK1
    MOVLW   0b01010111
    MOVWF   OPTION_REG

    ; GP2Y0D21 Enable.
    MOVLB   BANK2
    BSF	    LATC, GP2Y0D21_ENABLE
    ; Wait ~60ms.
    MOVLW   GP2Y0D21_60MS
    CALL    _delay

    ; INTERRUPTS Settings.
    BSF	    INTE
    BCF	    INTF
    ; INTERRUPTS Enabled.
    BSF	    GIE

    ; BATTERY Settings.
    MOVLW   BATTERY_FILTER
    MOVWF   filter

    ; TBOT Flags Clear.
    CLRF    TBOT

loop:
s0:
    ; SENSOR Obstacle ?
    MOVLB   BANK0
    BTFSC   PORTC, GP2Y0D21_OUT
    BRA	    rcServoSTOP
    ; TIMER0 For Battery Read & Filtering.
    BTFSC   TMR0IF
    BRA	    batteryRead
    ; TBOT RC Servo are Running ?
    BTFSC   TBOT, TBOT_RCSERVO
    BRA	    s0
    BRA	    rcServoFWD

; Battery Read & Filtering.
batteryRead:
    MOVLW   BATTERY_LOW
    MOVLB   BANK9
    SUBWF   ADRESH, W
    BTFSC   STATUS, C
    BRA	    $+5
    MOVLB   BANK0
    DECFSZ  filter, F
    BRA	    $+5
    BRA	    _emergency
    MOVLB   BANK0
    MOVLW   BATTERY_FILTER
    MOVWF   filter
    BCF     TMR0IF
    BRA     s0

; RC Servo Forward Ramp.
; TODO add Sensor Detection ?
rcServoFWD:
    MOVLB   BANK0
    MOVLW   SERVO_COUNTER
    MOVWF   counter
    MOVLB   BANK27
    MOVLW   SERVO_STOP_H
    MOVWF   PWM6DCH
    MOVWF   PWM11DCH
    MOVLW   SERVO_STOP_L
    MOVWF   PWM6DCL
    MOVWF   PWM11DCL
    MOVLW   0x6
    MOVWF   PWMLD
    MOVLB   BANK0
    DECFSZ  counter, F
    BRA	    $+2
    BRA	    $+19
    MOVLW   SERVO_MAX_RIGH_L
    MOVLB   BANK27
    XORWF   PWM6DCL, W
    BTFSS   STATUS, Z
    DECF    PWM6DCL, F
    MOVLW   SERVO_MAX_LEFT_L
    XORWF   PWM11DCL, W
    BTFSS   STATUS, Z
    INCF    PWM11DCL, F
    MOVLW   0x6
    MOVWF   PWMLD
    MOVLW   10 ; TODO delay
    call    _delay
    MOVLB   BANK27
    INCFSZ  PWM11DCL, W
    BRA	    $-19
    INCF    PWM11DCH, F
    BRA	    $-21
    BSF	    TBOT, TBOT_RCSERVO
    BRA	    s0

; RC Servo Stop.
rcServoSTOP:
    MOVLB   BANK0
    MOVLW   SERVO_COUNTER
    MOVWF   counter
    MOVLB   BANK0
    DECFSZ  counter, F
    BRA	    $+2
    BRA	    $+22
    MOVLB   BANK27
    MOVLW   SERVO_STOP_L
    XORWF   PWM6DCL, W
    BTFSS   STATUS, Z
    INCF    PWM6DCL, F
    MOVLW   SERVO_STOP_L
    XORWF   PWM11DCL, W
    BTFSS   STATUS, Z
    DECF    PWM11DCL, F
    MOVLW   0x6
    MOVWF   PWMLD
    MOVLW   4
    CALL    _delay ; TODO delay
    MOVLB   BANK27
    MOVLW   0
    XORWF   PWM11DCL, W
    BTFSS   STATUS, Z
    BRA	    $-21
    DECF    PWM11DCL, F
    DECF    PWM11DCH, F
    BRA	    $-24
    BCF	    TBOT, TBOT_RCSERVO



    BRA	    loop

; Interrupt Service Routine.
isr:
    ; Interrupt EMERGENCY ?
    BTFSC   INTF
    GOTO    _emergency
    RETFIE

; Functions.
; delay = 1 ~390us.
; delay = 255 ~98ms.
_delay:
    MOVLB   BANK0
    MOVWF   delay
    MOVLW   255
    DECFSZ  WREG, F
    BRA	    $-1
    DECFSZ  delay, F
    BRA	    $-4
    RETURN

_delay1:
    MOVLB   BANK0
    MOVWF   delay + 1
    movlw   255
    movwf   delay
    MOVLW   255
    DECFSZ  WREG, F
    BRA	    $-1
    DECFSZ  delay, F
    BRA	    $-4
    decfsz  delay + 1, F
    bra	    $-6
    RETURN

_emergency:
    BCF	    GIE
    MOVLB   BANK27
    CLRF    PWMEN
    MOVLB   BANK9
    BCF	    ADON
    MOVLB   BANK2
    BCF	    LATB, EMERGENCY
    MOVLB   BANK1
    BCF	    TRISB, EMERGENCY ; LED ON.
    MOVLW   255
    CALL    _delay
    MOVLB   BANK1
    BSF	    TRISB, EMERGENCY ; LED OFF.
    MOVLW   255
    CALL    _delay
    BRA	    $-8

    END	    resetVector
