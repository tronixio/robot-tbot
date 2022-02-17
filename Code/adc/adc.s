RETURN; Configuration Registers.
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
; Instruction ~500ns @8MHz.

; TBOT - ADC/TIMER0.
; EUSART TX - Display Battery value.

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
#define	BANK30  0x1E
#define	BANK31  0x1F

; User Definition.
; LED Debug.
#define	LED_DEBUG	0x6

; Reset Vector.
PSECT reset_vec,class=CODE,space=0,delta=2
resetVect:
    GOTO    main

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
    MOVLW   0b11111111
    MOVWF   SLRCONB
    MOVLW   0b11111111
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

    ; ADC Settings.
    ; TIMER0 Overflow.
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
    CALL    _debugBattery
    BRA	    loop

; Functions.
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
    MOVLW   10
    CALL    _delay
    MOVLW   0xd
    CALL    _eusartTX
    MOVLW   0xa
    CALL    _eusartTX
    MOVLW   '0'
    CALL    _eusartTX
    MOVLW   'x'
    CALL    _eusartTX
    MOVLB   BANK9
    MOVF    ADRESH, W
    CALL    _hex2ascii
    CALL    _eusartTX
    MOVLB   BANK9
    SWAPF   ADRESH, W
    CALL    _hex2ascii
    CALL    _eusartTX
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
