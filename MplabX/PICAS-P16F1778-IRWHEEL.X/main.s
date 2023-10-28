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
; PICAS-P16F1778-GP2Y0D21YKzz
; TODO: Better Delay Fonction
; TODO: Optimize Variables
; TODO: Faire le timming des boucles
; TODO: Utiliser qu un seul DAC pour les 2 comparateurs

; En focntion de la tension de la batterie
; choisir la bonne valeur pour que la vitesse
; soit constante en fonction de la tension de
; la battterie

; Pinout.
; MCU.RA6  -> LED.DEBUG / CLKOUTEN.OSCILLOSCOPE.PROBE.
; MCU.RB0 <-> INT0.EMERGENCY.LED.SWITCH.
; MCU.RB3 <-  AN9.BATTERY.SENSE.
; MCU.RB4 <-  CCP7.IR.WHEEL.LEFT.
; MCU.RB5  -> PWM11.RC.SERVO.LEFT.
; MCU.RB6  -> EUSART.TX.
; MCU.RB7 <-  EUSART.RX.
; MCU.RC0  -> GP2Y0D21YK.FRONT.ENABLE.
; MCU.RC2 <-  GP2Y0D21YK.FRONT.OUT.
; MCU.RC5  -> PWM6.RC.SERVO.RIGHT.
; MCU.RC6 <-  CCP2.IR.WHELL.RIGHT.

; GPR BANK0.
PSECT cstackBANK0,class=BANK0,space=1,delta=1
u16Delay:   DS  2
u8Filter:   DS	1
u8Speed:    DS	1
u8Rotation: DS  1

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
#define BATTERY_FILTER		50
#define	BATTERY_LOW		0xB7
; Emergency.
#define EMERGENCY		0x0
; LED Debug.
#define	LED_DEBUG		0x6
; RC Servo.
#define	SERVO_SPEED		202
; Frequency 50Hz - @8MHz.
#define SERVO_PERIOD_HIGH	78
#define SERVO_PERIOD_LOW	30
; Duty Cycle.
; H6/L165 - 1.7ms - @8MHz.
; H5/L220 - 1.5ms - @8MHz.
; H5/L20  - 1.3ms - @8MHz.
#define SERVO_STOP_HIGH		5
#define SERVO_STOP_LOW		220
;
#define SERVO_MAX_LEFT_LOW	165
#define SERVO_MAX_RIGHT_LOW	26
;
#define	SERVO_LEFT_LOAD		0x4
#define	SERVO_RIGHT_LOAD	0x2
#define	SERVO_LOAD		0x6
; Sharp GP2Y0D21YK.
#define	GP2Y0D21_60MS		160
#define GP2Y0D21_FRONT_ENABLE	0x0
#define GP2Y0D21_FRONT_OUT	0x2
; TBOT Flags.
#define TBOT_RCSERVO		0x0

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
    MOVLW   0b00001000
    MOVWF   TRISE
    ; LATCH Outputs.
    MOVLB   BANK2
    MOVLW   0b00000000
    MOVWF   LATA
    MOVLW   0b00000001
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
    ; PPS Inputs.
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
    ; TIMER0 Overflow Trigger.
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
    MOVLW   SERVO_STOP_LOW
    MOVWF   PWM6DCL
    MOVLW   SERVO_STOP_HIGH
    MOVWF   PWM6DCH
    MOVLW   SERVO_PERIOD_LOW
    MOVWF   PWM6PRL
    MOVLW   SERVO_PERIOD_HIGH
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
    MOVLW   SERVO_STOP_LOW
    MOVWF   PWM11DCL
    MOVLW   SERVO_STOP_HIGH
    MOVWF   PWM11DCH
    MOVLW   SERVO_PERIOD_LOW
    MOVWF   PWM11PRL
    MOVLW   SERVO_PERIOD_HIGH
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

    ; BATTERY Settings.
    MOVLW   BATTERY_FILTER
    MOVWF   u8Filter

    ; GP2Y0D21 Enable.
    MOVLB   BANK2
    BSF	    LATC, GP2Y0D21_FRONT_ENABLE
    ; Wait ~60ms.
    MOVLW   GP2Y0D21_60MS
    CALL    _delay
    ; GP2Y0D21 Obstacle ?
    BTFSC   PORTC, GP2Y0D21_FRONT_OUT
    BRA	    $-1

    ; INTERRUPTS Settings.
    BSF	    INTE
    BCF	    INTF
    ; INTERRUPTS Enabled.
    BSF	    GIE

    ;
    movlw   10 ; todo define
    movlb   BANK0
    movwf   u8Rotation
    
    ; TBOT Flags Clear.
    CLRF    TBOT

loop:
    ; GP2Y0D21 Obstacle ?
    MOVLB   BANK0
    BTFSC   PORTC, GP2Y0D21_FRONT_OUT
    BRA	    rcServoSTOP
    ; TIMER0 For Battery Read & Filtering.
    BTFSC   TMR0IF
    BRA	    batteryRead
    ; TBOT RC Servo are Running ?
    BTFSS   TBOT, TBOT_RCSERVO
    BRA	    rcServoFWD
    BRA	    loop

; Battery Read & Filtering.
batteryRead:
    MOVLW   BATTERY_LOW
    MOVLB   BANK9
    SUBWF   ADRESH, W
    BTFSC   STATUS, C
    BRA	    $+5
    MOVLB   BANK0
    DECFSZ  u8Filter, F
    BRA	    $+5
    BRA	    _emergency
    MOVLB   BANK0
    MOVLW   BATTERY_FILTER
    MOVWF   u8Filter
    BCF     TMR0IF
    BRA     loop

; RC Servo Forward Ramp.
; TODO add Sensor Detection ?
rcServoFWD:
    MOVLB   BANK0
    MOVLW   SERVO_SPEED
    MOVWF   u8Speed
    MOVLB   BANK27
    MOVLW   SERVO_STOP_LOW
    MOVWF   PWM6DCL
    MOVWF   PWM11DCL
    MOVLW   SERVO_STOP_HIGH
    MOVWF   PWM6DCH
    MOVWF   PWM11DCH
    MOVLW   SERVO_LOAD
    MOVWF   PWMLD
    MOVLB   BANK0
    DECFSZ  u8Speed, F
    BRA	    $+2
    BRA	    $+19
    MOVLW   SERVO_MAX_RIGHT_LOW
    MOVLB   BANK27
    XORWF   PWM6DCL, W
    BTFSS   STATUS, Z
    DECF    PWM6DCL, F
    MOVLW   SERVO_MAX_LEFT_LOW
    XORWF   PWM11DCL, W
    BTFSS   STATUS, Z
    INCF    PWM11DCL, F
    MOVLW   SERVO_LOAD
    MOVWF   PWMLD
    MOVLW   8 ; todo delay
    call    _delay
    MOVLB   BANK27
    INCFSZ  PWM11DCL, W
    BRA	    $-19
    INCF    PWM11DCH, F
    BRA	    $-21
    BSF	    TBOT, TBOT_RCSERVO
    BRA	    loop

; RC Servo Stop.
rcServoSTOP:
    MOVLB   BANK27
    MOVLW   SERVO_STOP_LOW
    MOVWF   PWM6DCL
    MOVWF   PWM11DCL
    MOVLW   SERVO_STOP_HIGH
    MOVWF   PWM6DCH
    MOVWF   PWM11DCH
    MOVLW   SERVO_LOAD
    MOVWF   PWMLD
    BCF	    TBOT, TBOT_RCSERVO
    MOVLB   BANK2
    BCF	    LATC, GP2Y0D21_FRONT_ENABLE
    BRA	    $

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
    MOVWF   u16Delay
    MOVLW   255
    DECFSZ  WREG, F
    BRA	    $-1
    DECFSZ  u16Delay, F
    BRA	    $-3
    RETURN

; delay = 1 ~98ms.
; delay = 255 ~25s.
_delay1:
    MOVLB   BANK0
    MOVWF   u16Delay
    movlw   255
    movwf   u16Delay + 1
    MOVLW   255
    DECFSZ  WREG, F
    BRA	    $-1
    DECFSZ  u16Delay + 1, F
    BRA	    $-3
    decfsz  u16Delay, F
    bra	    $-5
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



