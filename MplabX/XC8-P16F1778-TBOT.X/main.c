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

// faire un debug true / false

// remplacer le timer0 par un PWM pour declencher l'ADC

// NE CONSERVER QUE LED EUSART / MOTEUR / BATTERIE
// AJOUTER INT SECURITY

// comment demarrer les servo rapide puis descente
// ou commencer lentement pour augmenter
// se baser sur des capteurs de roue

// faire du ramping
// Pour le servo faire 2 fonctions.
// 1 pour preparer le LOAD des registres.
// 1 pour le BRAKE / STOP / START / RAMP.

// tbot_rcServoConfigure()
// tbot_rcServoSet()
// tbot_rcServoDO()

// Pinout.
// MCU.RA5 <-  ADC.AN4 (TIMER0) - BATTERY.
// MCU.RB0 <-> INT0 - URGENCY SWITCH.
// MCU.RB5 ->  PWM11.OUT - SERVO LEFT.
// MCU.RC1 <-  EUSART.URX - MCP2221.UTX.
// MCU.RC2 ->  EUSART.UTX - MCP2221.URX.
// MCU.RC3 ->  PWM5.OUT - HC-SR04 TRIGGER.
// MCU.RC5 ->  PWM6.OUT - SERVO RIGHT.

// Definitions.
// Battery.
#define BATTERY_LOW         180     // 240 ~8V2 / 180 ~6V2.
#define BATTERY_FILTER      75      // (1/15)*75 ~5s.
////#define BATTERY_ADC_CHANNEL 0x04
//// ASCII Characters.
#define ASCII_CR            0x0D
#define ASCII_B             0x42
// EUSART.
#define BAUDRATE            9600
#define BAUDRATE_GENERATOR  ((_XTAL_FREQ/BAUDRATE/64)-1)
// LED.
#define LED_1               0x7
#define LED_1_OFF           LATAbits.LATA7 = 0b0
#define LED_1_ON            LATAbits.LATA7 = 0b1
#define LED_2               0x6
#define LED_2_OFF           LATAbits.LATA6 = 0b0
#define LED_2_ON            LATAbits.LATA6 = 0b1
#define LED_URGENCY_OFF     TRISBbits.TRISB0 = 0b1
#define LED_URGENCY_ON      TRISBbits.TRISB0 = 0b0
// RC Servo.
// Frequency 50Hz - @8MHz.
#define SERVO_PERIOD_H      77
#define SERVO_PERIOD_L      180
// Duty Cycle.
// H6/L165 - 1.7ms - @8MHz.
// H5/L220 - 1.5ms - @8MHz.
// H5/L0   - 1.3ms - @8MHz.
#define SERVO_STOP_H        5
#define SERVO_STOP_L        220
#define SERVO_LEFT_L        PWM11DCL
#define SERVO_LEFT_H        PWM11DCH
#define SERVO_LEFT_STOP     PWM11CONbits.EN = 0b0
#define SERVO_LEFT_START    PWM11CONbits.EN = 0b1
#define SERVO_LEFT_LOAD     PWM11LDCONbits.LDA = 0b1
#define SERVO_RIGHT_L       PWM6DCL
#define SERVO_RIGHT_H       PWM6DCH
#define SERVO_RIGHT_STOP    PWM6CONbits.EN = 0b0
#define SERVO_RIGHT_START   PWM6CONbits.EN = 0b1
#define SERVO_RIGHT_LOAD    PWM6LDCONbits.LDA = 0b1

//// TBot Status Values.
//#define EMPTY   0
#define FULL    1
#define STOP    0
#define START   1
#define LOW     0

#define TRUE    1
#define FALSE   0
//#define HIGH    1

// Function Prototypes.
uint8_t eusart_readCharacter(void);
void eusart_writeCharacter(uint8_t u8Data);
void eusart_writeString(const uint8_t * u8Data);
void u16toa(uint16_t u16Data, uint8_t * au8Buffer, uint8_t u8Base);
// TBOT Functions Prototypes.
void tbot_batteryCheck(void);
void tbot_rcServo(void);

// Strings.
const uint8_t au8Tbot[] = "\r\n\r\nTBOT - v0.1";
const uint8_t au8Tronix[] = "\r\nTronix I/O";
const uint8_t au8WWW[] = "\r\nhttps://www.tronix.io/\r\n";
const uint8_t au8Ready[] = "\r\nREADY> ";
const uint8_t au8Error[] = "\r\nERROR!";
const uint8_t au8Battery[] = "\r\nBattery LOW !";

// Global Variables.
volatile uint8_t u8BatteryFilter = BATTERY_FILTER;

struct {
    uint8_t    BATTERY :1;
    uint8_t    MOTOR   :1;
    uint8_t    SPEED   :1;
    uint8_t    DUMMY   :5;
} TBOT_STATUS = {0,0,0,0};

// Interrupts Service Routines.
void __interrupt() ISR(void)
{
    LATA ^= (1 << LED_1);
    INTCONbits.TMR0IF = 0b0;
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
    TRISA = 0b00100000;
    TRISB = 0b00000101;
    TRISC = 0b00000010;
    TRISE = 0b00000000;
    // WPU Disable.
    OPTION_REGbits.nWPUEN = 0b1;
    // LATCH Outputs.
    LATA = 0b00000000;
    LATB = 0b00000000;
    LATC = 0b00000000;
    // ANSEL Analog.
    ANSELA = 0b00010000;
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
    RA0PPSbits.RA0PPS = 0b00011001;
    RB5PPSbits.RB5PPS = 0x1F;  // RB5 - PWM11.
    RC2PPSbits.RC2PPS = 0x24;  // RC2 - EUSART.UTX.
    RC5PPSbits.RC5PPS = 0x1E;  // RC5 - PWM6.
    PPSLOCK = 0x55;
    PPSLOCK = 0xAA;
    PPSLOCKbits.PPSLOCKED = 0b1;

    // ADC Settings.
    // Left Justified, Fosc/16.
    // TIMER0 Trigger Overflow.
    ADRESL = 0;
    ADRESH = 0;
    ADCON0 = 0x10;
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

    // TIMER0 Settings.
    // PS = 1 1 1 - 15Hz.
    OPTION_REGbits.TMR0CS = 0b0;
    OPTION_REGbits.PSA = 0b0;
    OPTION_REGbits.PS = 0b111;
    // TIMER0 Interrupt Disable.
    INTCONbits.TMR0IE = 0b0;
    INTCONbits.TMR0IF = 0b0;

    // PWM6 Settings.
    PWM6PHL = 0;
    PWM6PHH = 0;
    PWM6DCL = SERVO_STOP_L;
    PWM6DCH = SERVO_STOP_H;
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
    SERVO_RIGHT_LOAD;
    SERVO_RIGHT_STOP;

    // PWM11 Settings.
    PWM11PHL = 0;
    PWM11PHH = 0;
    PWM11DCL = SERVO_STOP_L;
    PWM11DCH = SERVO_STOP_H;
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
    SERVO_LEFT_LOAD;
    SERVO_LEFT_STOP;

    // Display Strings.
    eusart_writeString(au8Tbot);
    eusart_writeString(au8Tronix);
    eusart_writeString(au8WWW);
    eusart_writeString(au8Ready);

    // Interrupts Enable.
    INTCONbits.PEIE = 0b0;
    INTCONbits.GIE = 0b1;

    uint8_t au8Buffer[4], u8Rx;
    while(1){
       tbot_batteryCheck();
       
//        if(PIR1bits.RCIF){
//           u8Rx = eusart_readCharacter();
//           eusart_writeCharacter(u8Rx);
//           if(u8Rx == ASCII_CR)
//               eusart_writeString(au8Ready);
//        }
       
       LATA ^= (1 << LED_1);
        __delay_ms(200);
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

// TBOT Functions.
void tbot_batteryCheck(void)
{
    uint8_t au8Buffer[4];

    if(ADRESH < BATTERY_LOW) {
        if(INTCONbits.TMR0IF) {
            u8BatteryFilter--;
            INTCONbits.TMR0IF = 0b0;
            LATA ^= (1 << LED_1);
        }

        u16toa(u8BatteryFilter, au8Buffer, 10);
        eusart_writeString(au8Ready);
        eusart_writeString(au8Buffer);
        if(!u8BatteryFilter) {
            eusart_writeString(au8Battery);
            SERVO_LEFT_STOP;
            SERVO_RIGHT_STOP;
            TRISBbits.TRISB0 = 0b0;
            while(1){
                LATBbits.LATB0 = 0b0;
                __delay_ms(20);
                LATBbits.LATB0 = 0b1;
                __delay_ms(300);
           }
        }
    } else {
        u8BatteryFilter = BATTERY_FILTER;
    }
    u16toa(ADRESH, au8Buffer, 10);
    eusart_writeString(au8Ready);
    eusart_writeString(au8Buffer);
}

void tbot_rcServo(void)
{
    // Forward Full Speed  : L6/148 R5/0.
    // Forward Low Speed   : L6/40 R5/140.
    // Backward Full Speed : L5/0 R6/178.
    // Backward Low Speed  : L5/140 R6/50.
    if(TBOT_STATUS.MOTOR) {
        if(TBOT_STATUS.SPEED) {
            // Full Speed.
            SERVO_LEFT_L = 148;
            SERVO_LEFT_H = 6;
            SERVO_RIGHT_L = 0;
            SERVO_RIGHT_H = 5;
        } else {
            // Low Speed.
            SERVO_LEFT_L = 40;
            SERVO_LEFT_H = 6;
            SERVO_RIGHT_L = 140;
            SERVO_RIGHT_H = 5;
        }
        SERVO_LEFT_LOAD;
        SERVO_LEFT_START;
        SERVO_RIGHT_LOAD;
        SERVO_RIGHT_START;
    } else {
        // Stop.
        SERVO_LEFT_L = SERVO_STOP_L;
        SERVO_LEFT_H = SERVO_STOP_H;
        SERVO_RIGHT_L = SERVO_STOP_L;
        SERVO_RIGHT_H = SERVO_STOP_H;
        SERVO_LEFT_LOAD;
        SERVO_LEFT_START;
        SERVO_RIGHT_LOAD;
        SERVO_RIGHT_START;
    }
}