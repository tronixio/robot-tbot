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

; TBOT - v0.1.
; Blink LED.

; GPR BANK0.
PSECT cstackBANK0,class=BANK0,space=1,delta=1
delay:	    DS  3

; Common RAM.
PSECT cstackCOMM,class=COMMON,space=1,delta=1
ascii:	    DS  3
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
    movlw   0b00000000
    movwf   TRISA
    movlw   0b00000000
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
    movlw   0b00000000
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
    ; PPS Inputs.
    ; PPS Outputs.
    movlb   BANK29
    ; RC2 - EUSART.UTX.
    movlw   0x24
    movwf   RC2PPS
    ; PPS Write Disable.
    movlb   BANK28
    movlw   0x55
    movwf   PPSLOCK
    movlw   0xAA
    movwf   PPSLOCK
    bsf	    PPSLOCK, 0x0

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

    ; INTERRUPTS Enabled.
    bcf	    INTCON, PEIE
    bcf	    INTCON, GIE

loop:
    movlb   BANK2
    bcf	    LATA, 0x6
    movlw   5
    call    _delay
    movlb   BANK2
    bsf	    LATA, 0x6
    movlw   1
    call    _delay
    bra	    loop

; ISR
isr:
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

_hex2ascii:
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
    BTFSS   STATUS, C
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
