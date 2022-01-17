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
; PIC16F1778 - Compile with PIC-AS(v2.32).
; PIC16F1778 - @8MHz Internal Oscillator.
; -preset_vec=0000h, -pintentry=0004h, -pcinit=0005h, -pstringtext=3FC0h.
; Page0 0000h-07FFh - Page1 0800h-0FFFh - Page2 1000h-17FFh - Page3 1800h-1FFFh.
; Page4 2000h-27FFh - Page5 2800h-2FFFh - Page6 3000h-37FFh - Page7 3800h-3FFFh.
; 1 instruction 495ns.
; TIMER0 256/(8000000/(4*256)) = 0.032768s ~33ms (15Hz).

; TBOT - v0.1.
; FRONT SENSOR - SHARP GP2Y0D2
; Obstacle Avoidance.

; TODO : remplacer le flag.0 par le register IOCBF
; rempplacer flag par tbot_status
; TODO: le forwardStop ne peut pas etre appele pendant la rampe up
    ; a cause du compteur; donc cette rampe stop doit etre arrete
    ; quand le registre PWMxDCL est arrive a STOP
; TODO: Comments valeurs de BATTERIE
; TODO: Remetre le check batterie en service
    
; Pinout.
; MCU.RA5 <-  ADC.AN4 (TIMER0) - BATTERY.
; MCU.RB0 <-> INT0 - URGENCY SWITCH.
; MCU.RB1  -> GPIO - GP2Y0D2.ENABLE.
; MCU.RB2 <-  GPIO - GP2Y0D2.OUT.
; MCU.RB5  -> PWM11.OUT - SERVO LEFT.
;;;;;; MCU.RC1 <-  EUSART.URX - MCP2221.UTX.
; MCU.RC2  -> EUSART.UTX - MCP2221.URX.
;;;;;; MCU.RC4 <-  CCP1 (TIMER1) - HC-SR04 ECHO.
; MCU.RC5  -> PWM6.OUT - SERVO RIGHT.

; GPR BANK0.
PSECT cstackBANK0,class=BANK0,space=1,delta=1
battery:    DS  1
delay:	    DS  3
counter:    DS	1
turn:	    DS  1

; Common RAM.
PSECT cstackCOMM,class=COMMON,space=1,delta=1
ascii:	    DS  3
stringPTR:  DS  2
tick:	    DS	1
flag:	    DS	1

; FLAG Bits.
; FLAG.7 : 
; FLAG.6 : 
; FLAG.5 : 
; FLAG.4 : 
; FLAG.3 : 
; FLAG.2 : 
; FLAG.1 : 
; FLAG.0 : STOP FORWARD SLOW. changer pour un turn right

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
; SFR ADCON Bits.
#define GOnDONE 0x1
#define	ADON	0x0
; SFR INTCON Bits.
#define GIE	0x7
#define PEIE	0x6
#define TMR0IE	0x5
#define INTE	0x4
#define IOCIE	0x3
#define TMR0IF	0x2
#define INTF	0x1
#define IOCIF	0x0
; SFR OPTION_REG Bits.
#define nWPUEN	0x7
#define INTEDG	0x6
#define TMR0CS	0x5
#define TMR0SE	0x4
#define PSA	0x3
#define PS2	0x2
#define PS1	0x1
#define PS0	0x0
; SFR OSCSTAT Bits.
#define	SOSCR	0x7
#define	PLLR    0x6
#define	OSTS    0x5
#define	HFIOFR	0x4
#define	HFIOFL	0x3
#define	MFIOFR	0x2
#define	LFIOFR	0x1
#define	HFIOFS	0x0
; SFR RCSTA Bits.
#define	SPEN	0x7
#define RX9	0x6
#define	SREN	0x5
#define	CREN	0x4
#define	FERR	0x2
#define	OERR	0x1
#define	RX9D	0x0
; SFR STATUS Bits.
#define	nTO	0x4
#define	nPD	0x3
#define	Z	0x2
#define	DC	0x1
#define	C	0x0
; SFR PWMxCON.
#define	EN	0x7
; SFR PWMxLDCON.
#define	LDA	0x7

; User Definition.
; Battery.
#define BATTERY_LOW	191 ;
#define BATTERY_FILTER	0x5F
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
; Front Sensor.
; Sharp GP2Y0D2.
#define GP2Y0D2_ENABLE	0x1
#define GP2Y0D2_OUT	0x2

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
    btfss   OSCSTAT, HFIOFR
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
    movlw   0b00000101
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
    movlw   0b00010000
    movwf   ANSELA
    movlw   0b00000000
    movwf   ANSELB
    movlw   0b00000000
    movwf   ANSELC
    ; WPU Weak Pull-up.
    movlb   BANK4
    movlw   0b00000000
    movwf   WPUA
    movlw   0b00000100
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
    ; PPS Inputs.
    ; PPS Outputs.
    movlb   BANK29
    ; RB5 - PWM11.
    movlw   0x1F
    movwf   RB5PPS
    ; RC2 - EUSART.UTX.
    movlw   0x24
    movwf   RC2PPS
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

    ; ADC Settings.
    ; TIMER0 Overflow.
    ; Left Justified, FOSC/16.
    movlb   BANK9
    clrf    ADRESL
    clrf    ADRESH
    movlw   0x10
    movwf   ADCON0
    movlw   0x50
    movwf   ADCON1
    movlw   0x02
    movwf   ADCON2
    ; ADC Enable.
    bsf	    ADCON0, ADON

    ; EUSART Settings.
    ; 9600,8,N,1.
    movlb   BANK3
    clrf    RC1REG
    clrf    TX1REG
    movlw   12
    movwf   SP1BRGL
    clrf    SP1BRGH
    movlw   0x10
    movwf   RC1STA
    movlw   0x20
    movwf   TX1STA
    movlw   0x00
    movwf   BAUD1CON
    ; EUSART Enable.
    bsf	    RC1STA, SPEN

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
    bsf	    PWM6LDCON, LDA
    bcf	    PWM6CON, EN

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
    bsf	    PWM11LDCON, LDA
    bcf	    PWM11CON, EN

    ; EUSART String TRONIX.
    MOVLW   HIGH stringTRONIX + 0x80
    MOVWF   stringPTR
    MOVLW   LOW stringTRONIX
    MOVWF   stringPTR + 1
    CALL    _eusartTXString

    ; EUSART String URL.
    MOVLW   HIGH stringURL + 0x80
    MOVWF   stringPTR
    MOVLW   LOW stringURL
    MOVWF   stringPTR + 1
    CALL    _eusartTXString

    ; EUSART String TBOT.
    MOVLW   HIGH stringTBOT + 0x80
    MOVWF   stringPTR
    MOVLW   LOW stringTBOT
    MOVWF   stringPTR + 1
    CALL    _eusartTXString

    ; EUSART String READY.
    MOVLW   HIGH stringREADY + 0x80
    MOVWF   stringPTR
    MOVLW   LOW stringREADY
    MOVWF   stringPTR + 1
    CALL    _eusartTXString

    ; OPTION REG Settings.
    ; WPU Enabled.
    ; TIMER0 ~15Hz.
    movlb   BANK1
    movlw   0b01010111
    movwf   OPTION_REG

    ; INTERRUPTS Settings.
    bsf	    INTCON, TMR0IE
    bsf	    INTCON, INTE
    bcf	    INTCON, IOCIE
    bcf	    INTCON, TMR0IF
    bcf	    INTCON, INTF
    bcf	    INTCON, IOCIF
    ; INTERRUPTS Enabled.
    bsf	    INTCON, PEIE
    bsf	    INTCON, GIE

    ; TBOT BATTERY Settings.
    movlb   BANK0
    movlw   BATTERY_FILTER
    movwf   battery

    ; TBOT GP2Y0D2 Settings.
    movlb   BANK2
    bsf	    LATB, GP2Y0D2_ENABLE
    ; Wait ~65ms.
;    movlw   6
;    call    _delay
    clrf    tick
    movlw   2
    subwf   tick, W
    btfss   STATUS, C
    bra	    $-2
    bcf	    INTCON, TMR0IE
    movlb   BANK0
    btfsc   PORTB, GP2Y0D2_OUT
    bra	    $-1
    movlb   BANK7
    bsf	    IOCBP, GP2Y0D2_OUT
;    bsf	    IOCBN, GP2Y0D2_OUT
    clrf    IOCBF

    ; Clear Flag.
    CLRF    flag

loop:
    ; ici il faudra appeller le check battery

    call    _tbotForward
    btfss   flag, 0x0
    bra	    $-1
    bcf	    flag, 0x0
    call    _tbotForwardStop
    call    _tbotForwardLeft
    bra	    loop
    
foo:
    movlw   5
    call    _delay
    movlb   BANK2
    bsf	    LATA, 0x6
    movlw   5
    call    _delay
    movlb   BANK2
    bcf	    LATA, 0x6
    bra	    foo
    
    bra	    $

; ISR
isr:
    ; Interrupt Emergency ?
    BTFSC   INTCON, INTF
    BRA	    _tbotEmergency
    ; Interrupt Timer0 ?
    BTFSS   INTCON, TMR0IF
    BRA	    $+3
    INCF    tick, F
    BCF	    INTCON, TMR0IF
    ; Interrupt On Change ?
    btfss   INTCON, IOCIF
    retfie
    movlb   BANK7
    btfss   IOCBF, GP2Y0D2_OUT
    retfie
    bcf	    IOCBF, GP2Y0D2_OUT
    movlb   BANK0
    btfsc   PORTB, GP2Y0D2_OUT
    bsf	    flag, 0x0
    retfie

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

_delay1:
    movlb   BANK0
    movwf   delay + 1
    movlw   255
    movwf   delay
    decfsz  delay, F
    bra	    $-1
    decfsz  delay + 1, F
    bra	    $-5
    return

_eusartTX:
    MOVLB   BANK3
    MOVWF   TX1REG
    MOVLB   BANK0
    BTFSS   PIR1, 0x4
    BRA	    $-1
    RETURN

_eusartTXString:
    MOVF    stringPTR, W
    MOVWF   FSR0H
    MOVF    stringPTR + 1, W
    MOVWF   FSR0L
    MOVWI   0 [FSR0]
    MOVIW   FSR0++
    ANDLW   0xFF
    BTFSC   STATUS, Z
    RETURN
    CALL    _eusartTX
    BRA	    $-5

;_hex2ascii:
;    MOVWF   ascii + 2
;    ANDLW   0x0F
;    CALL    $+8
;    MOVWF   ascii + 1
;    SWAPF   ascii + 2, F
;    MOVF    ascii + 2, W
;    ANDLW   0x0F
;    CALL    $+3
;    MOVWF   ascii
;    RETURN
;    ; Decimal or Alpha ?
;    SUBLW   0x09
;    BTFSS   STATUS, C
;    BRA	    $+5
;    ; Decimal (0...9) add 0x30.
;    MOVF    ascii + 2, W
;    ANDLW   0x0F
;    ADDLW   0x30
;    RETURN
;    ; Alpha (A...F) add 0x37.
;    MOVF    ascii + 2, W
;    ANDLW   0x0F
;    ADDLW   'A' - 0x0A
;    RETURN

; TBOT Functions.
_tbotBatteryCheck:
    btfss   INTCON, TMR0IF
    RETURN
    bcf	    INTCON, TMR0IF
    ; ADRESH < BATTERY_LOW.
    movlb   BANK9
    movlw   BATTERY_LOW
    subwf   ADRESH, W
    btfsc   STATUS, C
    BRA	    $+5
    movlb   BANK0
    decfsz  battery, F
    RETURN
    GOTO    _tbotEmergency
    ; BATTERY Settings.
    movlb   BANK0
    movlw   BATTERY_FILTER
    movwf   battery
    RETURN

;_debugBattery:
;    movlw   10
;    call    _delay
;    RETURN
;    movlb   BANK2
;    bcf	    LATA, 0x7
;    movlw   0xd
;    call    _eusartTX
;    movlw   0xa
;    call    _eusartTX
;    movlw   '0'
;    call    _eusartTX
;    movlw   'x'
;    call    _eusartTX
;    movlb   BANK9
;    movf    ADRESH, W
;    call    _hex2ascii
;    call    _eusartTX
;    movlb   BANK9
;    swapf   ADRESH, W
;    call    _hex2ascii
;    call    _eusartTX
;    bcf	    INTCON, 0x2
;    movlw   10
;    call    _delay
;    RETURN

_tbotEmergency:
    ; INTERRUPTS Disable.
    BCF	    INTCON, GIE
    ; PWM6/11 Disable.
    MOVLB   BANK27
    CLRF    PWMEN
    ; Front Sensor Disable.
    MOVLB   BANK2
    BCF	    LATB, GP2Y0D2_ENABLE
    ; EUSART Disable.
    MOVLB   BANK3
    BCF	    RC1STA, SPEN
    ; ADC Disable.
    MOVLB   BANK9
    BCF	    ADCON0, ADON
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

_tbotForward:
    bsf	    INTCON, IOCIE
    movlb   BANK27
    movlw   SERVO_STOP_H
    movwf   PWM6DCH
    movwf   PWM11DCH
    movlW   SERVO_STOP_L
    movwf   PWM6DCL
    movwf   PWM11DCL
    movlw   0x06
    movwf   PWMLD
    movwf   PWMEN
    movlb   BANK0
    movlw   200
    movwf   counter
    movlb   BANK0
    decfsz  counter, F
    bra	    $+2
    bra	    $+13
    movlb   BANK27
    decf    PWM6DCL, F
    incf    PWM11DCL, F
    movlw   0x06
    movwf   PWMLD
    movlw   10
    call    _delay1
    movlb   BANK27
    incfsz  PWM11DCL, W
    bra	    $-13
    incf    PWM11DCH, F
    bra	    $-15
    return

_tbotForwardStop:
    movlb   BANK0
    movlw   200
    movwf   counter
    movlb   BANK0
    decfsz  counter, F
    bra	    $+2
    bra	    $+13
    movlb   BANK27
    incf    PWM6DCL, F
    decf    PWM11DCL, F
    movlw   0x06
    movwf   PWMLD
    movlw   5
    call    _delay1
    movlb   BANK27
    decfsz  PWM11DCL, W
    bra	    $-13
    decf    PWM11DCH, F
    bra	    $-15
    movlb   BANK27
    movlw   SERVO_STOP_H
    movwf   PWM6DCH
    movwf   PWM11DCH
    movlW   SERVO_STOP_L
    movwf   PWM6DCL
    movwf   PWM11DCL
    movlw   0x06
    movwf   PWMLD
    clrf    PWMEN
    bcf	    INTCON, IOCIE
    return

_tbotForwardLeft:
    bcf	    INTCON, IOCIE
    movlb   BANK0
    movlw   4
    movwf   turn
_tbotForwardLeft1:
    movlb   BANK27
    movlw   SERVO_STOP_H
    movwf   PWM6DCH
    movwf   PWM11DCH
    movlW   SERVO_STOP_L
    movwf   PWM6DCL
    movwf   PWM11DCL
    movlw   0x06
    movwf   PWMLD
    movwf   PWMEN
    movlb   BANK0
    movlw   60
    movwf   counter
    movlb   BANK0
    decfsz  counter, F
    bra	    $+2
    bra	    $+7
    movlb   BANK27
    decf    PWM6DCL, F
    bsf	    PWM6LDCON, LDA
    movlw   10
    call    _delay1
    bra	    $-9
    movlw   10
    call    _delay
    movlb   BANK0
    movlw   60
    movwf   counter
    decfsz  counter, F
    bra	    $+2
    bra	    $+5
    movlb   BANK27
    incf    PWM6DCL, F
    bsf	    PWM6LDCON, LDA
    bra	    $-6
    movlb   BANK0
    btfss   PORTB, GP2Y0D2_OUT
    return
    decfsz  turn, F
    BRA	    $-39
    BRA	    _tbotEmergency

; FPM Strings.
PSECT stringtext,class=STRCODE,space=0,delta=2
stringREADY:
    DB  0xD, 0xA, 0xD, 0xA, 'R','e','a','d','y','>',' ', 0x0

stringTBOT:
    DB  0xD, 0xA, 'T','B','O','T',' ','-',' ','v','0','.','1', 0x0

stringTRONIX:
    DB  0xD, 0xA, 0xD, 0xA, 'T','r','o','n','i','x',' ','I','/','O','.', 0x0

stringURL:
    DB  0xD, 0xA, 'w','w','w','.','t','r','o','n','i','x','.','c','o','m', 0x0

    END	    resetVect
