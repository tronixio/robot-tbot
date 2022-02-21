// Configuration Registers.
#pragma config FOSC = INTOSC, WDTE = OFF, PWRTE = OFF, MCLRE = ON, CP = OFF
#pragma config BOREN = OFF, CLKOUTEN = OFF, IESO = OFF, FCMEN = OFF
#pragma config WRT = OFF, PPS1WAY = ON, ZCD = OFF, PLLEN = OFF
#pragma config STVREN = ON, BORV = LO, LPBOR = OFF, LVP = ON

// MCU Frequency.
#define _XTAL_FREQ 8000000

#include <xc.h>
#include <stdint.h>
// PIC16F1778 - Compile with XC8(v2.35).
// PIC16F1778 - @8MHz Internal Oscillator.
// v0.1 - xx/2022.

// TBOT - Rev.A.
// Debug:
//  (B)attery: .
//  (L)ED: Toggle Debug LED.
//  (M)otors: Toggle RC Servos Start / Stop.
//  (S)ensors: .

// Definitions.
//// ASCII Characters.
#define ASCII_CR            0x0D
#define ASCII_B             0x42
#define ASCII_L             0x4C
#define ASCII_M             0x4D
#define ASCII_S             0x53
// EUSART.
#define BAUDRATE            9600
#define BAUDRATE_GENERATOR  ((_XTAL_FREQ/BAUDRATE/64)-1)
// LED.
#define LED_DEBUG           LATAbits.LATA6
// RC Servo.
// Frequency 50Hz - @8MHz.
#define SERVO_PERIOD_H	78
#define SERVO_PERIOD_L  30
// Duty Cycle.
// H6/L165 - 1.7ms - @8MHz.
// H5/L220 - 1.5ms - @8MHz.
// H5/L20  - 1.3ms - @8MHz.
#define SERVO_STOP_H	5
#define SERVO_STOP_L	220
// Sharp GP2Y0D21YK.
#define GP2Y0D21_ENABLE	LATCbits.LATC0
#define GP2Y0D21_OUT	PORTCbits.RC2

// Function Prototypes.
uint8_t eusart_readCharacter(void);
void eusart_writeCharacter(uint8_t u8Data);
void eusart_writeString(const uint8_t * u8Data);
void u16toa(uint16_t u16Data, uint8_t * au8Buffer, uint8_t u8Base);

// Strings.
const uint8_t au8Battery[] = " - Battery ";
const uint8_t au8LED[] = " - LED Toggle";
const uint8_t au8Tbot[] = "\r\nTBOT - v0.1 - Firmware Debug.";
const uint8_t au8Tronix[] = "\r\n\r\nTronix I/O.";
const uint8_t au8WWW[] = "\r\nwww.tronix.io";
const uint8_t au8RCServo[] = " - RC Servo Toggle";
const uint8_t au8Ready[] = "\r\n\r\nREADY> ";
const uint8_t au8Sensor[] = " - Sensor ";
const uint8_t au8SensorObstacle[] = "Obstacle";
const uint8_t au8SensorNOObstacle[] = "No Obstacle";

// Main.
void main(void)
{
    // MCU Initialization.
    // Internal Oscillator Settings.
    OSCCON = 0x70;
    while(!OSCSTATbits.HFIOFR){};
    // Ports Settings.
    // PORT Data Register.
    PORTA = 0b00000000;
    PORTB = 0b00000000;
    PORTC = 0b00000000;
    PORTE = 0b00000000;
    // TRIS Data Direction.
    TRISA = 0b00000000;
    TRISB = 0b10001001;
    TRISC = 0b00000100;
    TRISE = 0b00000000;
    // LATCH Outputs.
    LATA = 0b00000000;
    LATB = 0b00000000;
    LATC = 0b00000000;
    // ANSEL Analog.
    ANSELA = 0b00000000;
    ANSELB = 0b00001000;
    ANSELC = 0b00000000;
    // WPU Weak Pull-up.
    WPUA = 0b00000000;
    WPUB = 0b00000000;
    WPUC = 0b00000100;
    WPUE = 0b00000000;
    // ODCON Open-drain.
    ODCONA = 0b00000000;
    ODCONB = 0b00000000;
    ODCONC = 0b00000000;
    // SRLCON Slew Rate.
    SLRCONA = 0b11111111;
    SLRCONB = 0b11011111;
    SLRCONC = 0b11011111;
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
    RXPPSbits.RXPPS = 0x0F;     // RB7 - EUSART.URX.
    // PPS Outputs.
    RB5PPSbits.RB5PPS = 0x1F;   // RB5 - PWM11.
    RB6PPSbits.RB6PPS = 0x24;   // RB6 - EUSART.UTX.
    RC5PPSbits.RC5PPS = 0x1E;   // RC5 - PWM6.
    PPSLOCK = 0x55;
    PPSLOCK = 0xAA;
    PPSLOCKbits.PPSLOCKED = 0b1;

    // ADC Settings.
    // TIMER0 Overflow Trigger.
    // Left Justified, FOSC/16.
    ADRESL = 0;
    ADRESH = 0;
    ADCON0 = 0x24;
    ADCON1 = 0x50;
    ADCON2 = 0x02;
    // ADC Enable.
    ADCON0bits.ADON = 0b1;

    // EUSART Settings.
    RC1REG = 0;
    TX1REG = 0;
    SP1BRG = BAUDRATE_GENERATOR;
    RC1STA = 0x10;
    TX1STA = 0x20;
    BAUD1CON = 0x00;
    // EUSART Enable.
    RC1STAbits.SPEN = 0b1;

    // OPTION REG Settings.
    //WPU Enabled.
    // TIMER0 ~15Hz.
    OPTION_REG = 0b01010111;

    // PWM6 Settings.
    PWM6PHL = 0;
    PWM6PHH = 0;
    PWM6DCL = 120;
    PWM6DCH = 5;
    PWM6PRL = SERVO_PERIOD_L;
    PWM6PRH = SERVO_PERIOD_H;
    PWM6OFL = 0;
    PWM6OFH = 0;
    PWM6TMRL = 0;
    PWM6TMRH = 0;
    PWM6CON = 0x0C;
    PWM6INTE = 0x00;
    PWM6INTF = 0x00;
    PWM6CLKCON = 0x20;
    PWM6LDCON = 0x00;
    PWM6OFCON = 0x00;
    PWM6LDCONbits.LDA = 0b1;
    PWM6CONbits.EN = 0b0;

    // PWM11 Settings.
    PWM11PHL = 0;
    PWM11PHH = 0;
    PWM11DCL = 60;
    PWM11DCH = 6;
    PWM11PRL = SERVO_PERIOD_L;
    PWM11PRH = SERVO_PERIOD_H;
    PWM11OFL = 0;
    PWM11OFH = 0;
    PWM11TMRL = 0;
    PWM11TMRH = 0;
    PWM11CON = 0x0C;
    PWM11INTE = 0x00;
    PWM11INTF = 0x00;
    PWM11CLKCON = 0x20;
    PWM11LDCON = 0x00;
    PWM11OFCON = 0x00;
    PWM11LDCONbits.LDA = 0b1;
    PWM11CONbits.EN = 0b0;

    // GP2Y0D21 Settings.
    GP2Y0D21_ENABLE = 0b1;
    // Wait ~60ms.
    __delay_ms(60);

    // Display Strings.
    eusart_writeString(au8Tronix);
    eusart_writeString(au8WWW);
    eusart_writeString(au8Tbot);
    eusart_writeString(au8Ready);

    uint8_t u8Rx;
    uint8_t au8Buffer[6];
    while(1){
        if(PIR1bits.RCIF){
            u8Rx = eusart_readCharacter();
            eusart_writeCharacter(u8Rx);
            // Battery.
            if(u8Rx == ASCII_B){
                eusart_writeString(au8Battery);
                u16toa(ADRESH, au8Buffer, 10);
                eusart_writeString(au8Buffer);
                eusart_writeString(au8Ready);
            }
            // LED Debug Toggle.
            if(u8Rx == ASCII_L){
                LED_DEBUG = ~LED_DEBUG;
                eusart_writeString(au8LED);
                eusart_writeString(au8Ready);
            }
            // Motors Toggle.
            if(u8Rx == ASCII_M){
                PWM6CONbits.EN = ~PWM6CONbits.EN;
                PWM11CONbits.EN = ~PWM11CONbits.EN;
                eusart_writeString(au8RCServo);
                eusart_writeString(au8Ready);
            }
            // Sensor Read.
            if(u8Rx == ASCII_S){
                eusart_writeString(au8Sensor);
                if(GP2Y0D21_OUT){
                    eusart_writeString(au8SensorObstacle);
                }else{
                    eusart_writeString(au8SensorNOObstacle);
                }
                eusart_writeString(au8Ready);
            }
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
