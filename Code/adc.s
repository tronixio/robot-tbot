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
; -preset_vec=0000h, -pcinit=0005h, -pstringtext=3FC0h.
; 1 instruction 495ns.

; TBOT - v0.1.

; GPR BANK0.
PSECT cstackBANK0,class=BANK0,space=1,delta=1
ascii:	    DS  3
delay:	    DS  3
stringPTR:  DS  2

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

; Reset Vector.
PSECT reset_vec,class=CODE,space=0,delta=2
resetVect:
    goto    main

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
    movlw   0b11111111
    movwf   SLRCONB
    movlw   0b11111111
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
    ; RB6 - EUSART.UTX.
    movlw   0x24
    movwf   RB6PPS
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
    movlw   0x24
    movwf   ADCON0
    movlw   0x50
    movwf   ADCON1
    movlw   0x02
    movwf   ADCON2
    ; ADC Enable.
    bsf	    ADON

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
    bsf	    SPEN
    ; OPTION REG Settings.
    ; WPU Enabled.
    ; TIMER0 ~15Hz.
    MOVLB   BANK1
    MOVLW   0b01010111
    MOVWF   OPTION_REG

    ; EUSART String TRONIX.
    MOVLB   BANK0
    MOVLW   HIGH stringTRONIX + 0x80
    MOVWF   stringPTR
    MOVLW   LOW stringTRONIX
    MOVWF   stringPTR + 1
    CALL    _eusartTXString

    ; EUSART String URL.
    MOVLB   BANK0
    MOVLW   HIGH stringURL + 0x80
    MOVWF   stringPTR
    MOVLW   LOW stringURL
    MOVWF   stringPTR + 1
    CALL    _eusartTXString

    ; EUSART String TBOT.
    MOVLB   BANK0
    MOVLW   HIGH stringTBOT + 0x80
    MOVWF   stringPTR
    MOVLW   LOW stringTBOT
    MOVWF   stringPTR + 1
    CALL    _eusartTXString

    ; EUSART String READY.
    MOVLB   BANK0
    MOVLW   HIGH stringREADY + 0x80
    MOVWF   stringPTR
    MOVLW   LOW stringREADY
    MOVWF   stringPTR + 1
    CALL    _eusartTXString

loop:
    call    _debugBattery
    bra	    loop

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

_eusartTX:
    MOVLB   BANK3
    MOVWF   TX1REG
    MOVLB   BANK0
    BTFSS   TXIF
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
    BTFSC   ZERO
    RETURN
    CALL    _eusartTX
    BRA	    $-5

_hex2ascii:
    MOVLB   BANK0
    MOVWF   ascii + 2
    ANDLW   0x0F
    CALL    $+8
    MOVWF   ascii + 1
    SWAPF   ascii + 2, F
    MOVF    ascii + 2, W
    ANDLW   0x0F
    CALL    $+3
    MOVWF   ascii
    RETURN
    ; Decimal or Alpha ?
    SUBLW   0x09
    BTFSS   CARRY
    BRA	    $+5
    ; Decimal (0...9) add 0x30.
    MOVF    ascii + 2, W
    ANDLW   0x0F
    ADDLW   0x30
    RETURN
    ; Alpha (A...F) add 0x37.
    MOVF    ascii + 2, W
    ANDLW   0x0F
    ADDLW   'A' - 0x0A
    RETURN

_debugBattery:
    movlw   10
    call    _delay
    movlw   0xd
    call    _eusartTX
    movlw   0xa
    call    _eusartTX
    movlw   '0'
    call    _eusartTX
    movlw   'x'
    call    _eusartTX
    movlb   BANK9
    movf    ADRESH, W
    call    _hex2ascii
    call    _eusartTX
    movlb   BANK9
    swapf   ADRESH, W
    call    _hex2ascii
    call    _eusartTX
    RETURN

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
