// Configuration Registers.
#pragma config FOSC = INTOSC, WDTE = OFF, PWRTE = OFF, MCLRE = ON, CP = OFF
#pragma config BOREN = OFF, CLKOUTEN = OFF, IESO = OFF, FCMEN = OFF
#pragma config WRT = OFF, PPS1WAY = ON, ZCD = OFF, PLLEN = OFF
#pragma config STVREN = ON, BORV = LO, LPBOR = OFF, LVP = ON

// MCU Frequency.
#define _XTAL_FREQ 8000000

#include <xc.h>
#include <stdint.h>
// PIC16F1778 - Compile with XC8(v2.32).
// PIC16F1778 - @8MHz Internal Oscillator.
// v0.1 - xx/2021.

// TBOT - Rev.A.
// Blink LED.

// Definitions.
//// ASCII Characters.
#define ASCII_CR            0x0D
#define ASCII_B             0x42
// EUSART.
#define BAUDRATE            9600
#define BAUDRATE_GENERATOR  ((_XTAL_FREQ/BAUDRATE/64)-1)
// LED.
#define LED_1               LATAbits.LATA6

// Function Prototypes.
uint8_t eusart_readCharacter(void);
void eusart_writeCharacter(uint8_t u8Data);
void eusart_writeString(const uint8_t * u8Data);
void u16toa(uint16_t u16Data, uint8_t * au8Buffer, uint8_t u8Base);

// Strings.
const uint8_t au8Tbot[] = "\r\n\r\nTBOT - v0.1";
const uint8_t au8Tronix[] = "\r\nTronix I/O";
const uint8_t au8WWW[] = "\r\nhttps://www.tronix.io/\r\n";
const uint8_t au8Ready[] = "\r\nREADY> ";
const uint8_t au8Error[] = "\r\nERROR!";

// Global Variables.

// Interrupts Service Routines.
void __interrupt() ISR(void)
{
}

// Main.
void main(void)
{
    // MCU Initialization.
    // Internal Oscillator Settings.
    OSCCON = 0x70;
    // Ports Settings.
    // PORT Data Register.
    PORTA = 0b00000000;
    PORTB = 0b00000000;
    PORTC = 0b00000000;
    PORTE = 0b00000000;
    // TRIS Data Direction.
    TRISA = 0b00000000;
    TRISB = 0b00000000;
    TRISC = 0b00000000;
    TRISE = 0b00000000;
    // WPU Disable.
    OPTION_REGbits.nWPUEN = 0b1;
    // LATCH Outputs.
    LATA = 0b00000000;
    LATB = 0b00000000;
    LATC = 0b00000000;
    // ANSEL Analog.
    ANSELA = 0b00000000;
    ANSELB = 0b00000000;
    ANSELC = 0b00000000;
    // WPU Weak Pull-up.
    WPUA = 0b00000000;
    WPUB = 0b00000000;
    WPUC = 0b00000000;
    WPUE = 0b00000000;
    // ODCON Open-drain.
    ODCONA = 0b00000000;
    ODCONB = 0b00000000;
    ODCONC = 0b00000000;
    // SRLCON Slew Rate.
    SLRCONA = 0b11111111;
    SLRCONB = 0b11111111;
    SLRCONC = 0b11111111;
    // INLVL Input Level.
    INLVLA  = 0b00000000;
    INLVLB  = 0b00000000;
    INLVLC  = 0b00000000;
    // HIDRVB High Drive.
    HIDRVB  = 0b00000000;
    // PPS Settings.
    PPSLOCK = 0x55;
    PPSLOCK = 0xAA;
    PPSLOCKbits.PPSLOCKED = 0b0;
    // PPS Inputs.
    RXPPSbits.RXPPS = 0x11;     // RC1 - EUSART.URX.
    // PPS Outputs.
    RC2PPSbits.RC2PPS = 0x24;  // RC2 - EUSART.UTX.
    PPSLOCK = 0x55;
    PPSLOCK = 0xAA;
    PPSLOCKbits.PPSLOCKED = 0b1;

    // EUSART Settings.
    RC1REG = 0;
    TX1REG = 0;
    SP1BRG = BAUDRATE_GENERATOR;
    RC1STA = 0x10;
    TX1STA = 0x20;
    BAUD1CON = 0x00;
    // EUSART Enable.
    RC1STAbits.SPEN = 0b1;

    // Display Strings.
    eusart_writeString(au8Tbot);
    eusart_writeString(au8Tronix);
    eusart_writeString(au8WWW);
    eusart_writeString(au8Ready);

    // Interrupts Enable.
    INTCONbits.PEIE = 0b0;
    INTCONbits.GIE = 0b0;

    while(1){
        
        if(0xdf>ADRESH){
        LED_1 = 0;
        __delay_ms(200);
        LED_1 = 1;
        __delay_ms(200);
        }
    }
}

// Functions.
uint8_t eusart_readCharacter(void)
{
    if(RC1STAbits.OERR){
        RC1STAbits.CREN = 0b0;
        RC1STAbits.CREN = 0b1;
    }

    while(!PIR1bits.RCIF){};
    return(RC1REG);
}

void eusart_writeCharacter(uint8_t u8Data)
{
    while(!PIR1bits.TXIF){};
    TX1REG = u8Data;
}

void eusart_writeString(const uint8_t * u8Data)
{
    while(*u8Data != '\0')
        eusart_writeCharacter(*u8Data++);
}

void u16toa(uint16_t u16Data, uint8_t * au8Buffer, uint8_t u8Base)
{
    uint8_t u8Buffer;
    uint16_t data = u16Data;

    while(data != '\0'){
        data /= u8Base;
        au8Buffer++;
    }
    *au8Buffer-- = 0;

    while(u16Data != '\0'){
        u8Buffer = u16Data % u8Base;
        u16Data /= u8Base;
        if(u8Buffer >= 10)
            u8Buffer += 'A' - '0' - 10;
        u8Buffer += '0';
        *au8Buffer-- = u8Buffer;
    }
}
