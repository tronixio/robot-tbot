// Configuration Registers.
#pragma config FOSC = INTOSC, WDTE = OFF, PWRTE = OFF, MCLRE = ON, CP = OFF
#pragma config BOREN = OFF, CLKOUTEN = OFF, IESO = OFF, FCMEN = OFF
#pragma config WRT = OFF, PPS1WAY = ON, ZCD = OFF, PLLEN = OFF
#pragma config STVREN = ON, BORV = LO, LPBOR = OFF, LVP = ON

#include <xc.h>
#include <stdint.h>
#define _XTAL_FREQ 8000000
// PIC16F1778 - Compile with XC8(v2.36).
// PIC16F1778 - @8MHz Internal Oscillator.
// Instruction ~500ns @8MHz.

// Pinout.
// MCU.RA6  -> LED.DEBUG / CLKOUTEN.OSCILLOSCOPE.PROBE.
// MCU.RB0 <-> INT0.EMERGENCY.LED.SWITCH.
// MCU.RB3 <-  AN9.BATTERY.SENSE.
// MCU.RB4 <-  CCP7.IR.WHEEL.LEFT.
// MCU.RB5  -> PWM11.RC.SERVO.LEFT.
// MCU.RB6  -> EUSART.TX.
// MCU.RB7 <-  EUSART.RX.
// MCU.RC0  -> GP2Y0D21YK.FRONT.ENABLE.
// MCU.RC2 <-  GP2Y0D21YK.FRONT.OUT.
// MCU.RC5  -> PWM6.RC.SERVO.RIGHT.
// MCU.RC6 <-  CCP2.IR.WHELL.RIGHT.

// Definitions.
// Battery.
#define BATTERY_FILTER		50
#define	BATTERY_LOW		0xB7
// Emergency.
#define EMERGENCY		0x0
// LED Debug.
#define	LED_DEBUG_OFF    LATAbits.LATA6 = 0b0;
#define	LED_DEBUG_ON    LATAbits.LATA6 = 0b1;
// RC Servo.
#define	SERVO_SPEED		202
// Frequency 50Hz - @8MHz.
#define SERVO_PERIOD_HIGH	78
#define SERVO_PERIOD_LOW	30
// Duty Cycle.
// H6/L165 - 1.7ms - @8MHz.
// H5/L220 - 1.5ms - @8MHz.
// H5/L20  - 1.3ms - @8MHz.
#define SERVO_STOP_HIGH		5
#define SERVO_STOP_LOW		220
//
#define SERVO_MAX_LEFT_LOW	165
#define SERVO_MAX_RIGHT_LOW	26
//
#define	SERVO_LEFT_LOAD		0x4
#define	SERVO_RIGHT_LOAD	0x2
#define	SERVO_LOAD		0x6
// Sharp GP2Y0D21YK.
#define	GP2Y0D21_60MS		60
#define GP2Y0D21_FRONT_DISABLE	LATCbits.LATC0 = 0b0
#define GP2Y0D21_FRONT_ENABLE	LATCbits.LATC0 = 0b1
#define GP2Y0D21_FRONT_OUT	PORTCbits.RC2

// Function Prototypes.
uint8_t eusart_readCharacter(void);
void eusart_writeCharacter(uint8_t u8Data);
void eusart_writeString(const uint8_t * u8Data);
void u16toa(uint16_t u16Data, uint8_t * au8Buffer, uint8_t u8Base);

// Strings.
const uint8_t au8Tronix[] = "\r\n\r\nTronix I/O";
const uint8_t au8WWW[] = "\r\nhttps://www.tronix.io/\r\n";
const uint8_t au8Ready[] = "\r\nREADY> ";

// Interrupts Service Routines.
void __interrupt() ISR(void)
{
//    if(PIR1bits.TMR1IF){
//        LATAbits.LATA6 = ~LATAbits.LATA6;
//        PIR1bits.TMR1IF = 0b0;
//    }
    if(INTCONbits.INTF){
        INTCONbits.GIE = 0;
        PWMEN = 0;
//        ADCON0bits.ADON = 0b0;
//        RC1STAbits.SPEN = 0b0;
//        T1CONbits.ON = 0b0;
        LATBbits.LATB0 = 0b0;
        TRISBbits.TRISB0 = 0b0;
        while(1){};
    }
    if(PIR4bits.TMR3IF){
        PWM6CONbits.EN = 0;
        PIR4bits.TMR3IF = 0;
//        while(1){};
    }
    if(PIR4bits.TMR5IF){
        PWM11CONbits.EN = 0;
        PIR4bits.TMR5IF = 0;
//        while(1){};
    }
}

// Main.
void main(void)
{
    // MCU Initialization.
    // Internal Oscillator Settings.
    OSCTUNE = 0b00000110;
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
    TRISB = 0b10011001;
    TRISC = 0b01010100;
    TRISE = 0b00001000;
    // WPU Disable.
//    OPTION_REGbits.nWPUEN = 0b1;
    // LATCH Outputs.
    LATA = 0b00000000;
    LATB = 0b00000001;
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
    // PPS Enabled Writes.
    PPSLOCK = 0x55;
    PPSLOCK = 0xAA;
    PPSLOCKbits.PPSLOCKED = 0b0;
    // PPS Inputs.
    RXPPSbits.RXPPS = 0x0F;         // RB7 - EUSART.URX.
    T3GPPSbits.T3GPPS = 0x14;       // RC4
    T3CKIPPSbits.T3CKIPPS = 0x14;   // RC4
    T5GPPSbits.T5GPPS = 0x16;       // RC6
    T5CKIPPSbits.T5CKIPPS = 0x16;   // RC6
    // PPS Outputs.
    RA0PPSbits.RA0PPS = 0x1D;   // RA0 - PWM5.
    RB5PPSbits.RB5PPS = 0x1F;   // RB5 - PWM11.
    RB6PPSbits.RB6PPS = 0x24;   // RB6 - EUSART.UTX.
    RC5PPSbits.RC5PPS = 0x1E;   // RC5 - PWM6.
    // PPS Disabled Writes.
    PPSLOCK = 0x55;
    PPSLOCK = 0xAA;
    PPSLOCKbits.PPSLOCKED = 0b1;

    // ADC Settings.
    ADRESL = 0;
    ADRESH = 0;
    ADCON0 = 0x24;
    ADCON1 = 0x50;
    ADCON2 = 0x02;
    // ADC Enable.
    ADCON0bits.ADON = 0b0;

    // EUSART Settings.
    // 9600,8,N,1.
    RC1REG = 0;
    TX1REG = 0;
    SP1BRGL = 12;
    SP1BRGH = 0;
    RC1STA = 0x10;
    TX1STA = 0x20;
    BAUD1CON = 0x00;
    // EUSART Enable.
    RC1STAbits.SPEN = 0b1;

    // PWM5 Settings.
    // Test Signal.
    PWM5PHL = 0b00000000;
    PWM5PHH = 0b00000000;
    PWM5DCL = 0;
    PWM5DCH = 9;
    PWM5PRL = 0;
    PWM5PRH = 18;
    PWM5OFL = 0b00000000;
    PWM5OFH = 0b00000000;
    PWM5TMRL = 0b00000000;
    PWM5TMRH = 0b00000000;
    PWM5CON = 0x0C;
    PWM5INTE = 0x00;
    PWM5INTF = 0x00;
    PWM5CLKCON = 0x20;
    PWM5LDCON = 0x00;
    PWM5OFCON = 0x00;
    // PWM5 Load & Enable.
    PWM5LDCONbits.LDA = 0b0;
    PWM5CONbits.EN = 0b0;

    // PWM6 Settings.
    // RC Servo Right
    PWM6PHL = 0b00000000;
    PWM6PHH = 0b00000000;
    PWM6DCL = SERVO_STOP_LOW;
    PWM6DCH = SERVO_STOP_HIGH;
    PWM6PRL = SERVO_PERIOD_LOW;
    PWM6PRH = SERVO_PERIOD_HIGH;
    PWM6OFL = 0b00000000;
    PWM6OFH = 0b00000000;
    PWM6TMRL = 0b00000000;
    PWM6TMRH = 0b00000000;
    PWM6CON = 0x0C;
    PWM6INTE = 0x00;
    PWM6INTF = 0x00;
    PWM6CLKCON = 0x20;
    PWM6LDCON = 0x00;
    PWM6OFCON = 0x00;
    // PWM6 Load & Enable.
    PWM6LDCONbits.LDA = 0b1;
    PWM6CONbits.EN = 0b1;
    
    // PWM11 Settings.
    // RC Servo Left
    PWM11PHL = 0b00000000;
    PWM11PHH = 0b00000000;
    PWM11DCL = SERVO_STOP_LOW;
    PWM11DCH = SERVO_STOP_HIGH;
    PWM11PRL = SERVO_PERIOD_LOW;
    PWM11PRH = SERVO_PERIOD_HIGH;
    PWM11OFL = 0b00000000;
    PWM11OFH = 0b00000000;
    PWM11TMRL = 0b00000000;
    PWM11TMRH = 0b00000000;
    PWM11CON = 0x0C;
    PWM11INTE = 0x00;
    PWM11INTF = 0x00;
    PWM11CLKCON = 0x20;
    PWM11LDCON = 0x00;
    PWM11OFCON = 0x00;
    // PWM11 Load & Enable.
    PWM11LDCONbits.LDA = 0b1;
    PWM11CONbits.EN = 0b1;

    // TIMER3 Settings.
    TMR3L = 250;
    TMR3H = 255;
    T3CON = 0b10000000;
    T3GCON = 0b11100000;
    // TIMER3 Enable.
    T3CONbits.ON = 0b1;
    PIE4bits.TMR3IE = 1;
    PIR4bits.TMR3IF = 0;

    // TIMER5 Settings.
    TMR5L = 200;
    TMR5H = 255;
    T5CON = 0b10000000;
    T5GCON = 0b11100000;
    // TIMER5 Enable.
    T5CONbits.ON = 0b1;
    PIE4bits.TMR5IE = 1;
    PIR4bits.TMR5IF = 0;

    // OPTION REG Settings.
    // WPU Enabled.
    // TIMER0 ~15Hz @8MHz.
    OPTION_REG = 0b01010111;

    // Timer1 Settings.
    TMR1L = 0;
    TMR1H = 0;
    T1CON = 0b00110000;
    T1GCON = 0b00000000;
    // Timer1 Enable.
    PIE1bits.TMR1IE = 0b0;
    PIR1bits.TMR1IF = 0b0;
    T1CONbits.ON = 0b0;

    // GP2Y0D21 Enable.
//    GP2Y0D21_FRONT_ENABLE;
    // Wait ~60ms.
//    __delay_ms(GP2Y0D21_60MS);
    // GP2Y0D21 Obstacle ?
//    while(GP2Y0D21_FRONT_OUT){};

    //
    INTCONbits.INTE = 0b1;
    INTCONbits.INTF = 0b0;
    // Interrupts Enable.
    INTCONbits.PEIE = 0b0;
    INTCONbits.GIE = 0b1;

    // Display Strings.
    eusart_writeString(au8Tronix);
    eusart_writeString(au8Ready);

    PWM6DCL = 220; // 0; // back speed up -> 165 - RIGHT
    PWM6DCH = 5; // #6; // RIGHT
    PWM11DCL = 220; //180; // back speed up -> 26 - LEFT
    PWM11DCH = 5; // LEFT
    PWMLD = 6; // 0x2 right, 0x4 left, 0x6 both
    PWMEN = 6;

    uint16_t u16Result1 = 0;
    uint16_t u16Result2 = 0;
    uint8_t au8Buffer[6];
    while(1){
        TMR3L = 0;
        TMR3H = 0;
        TMR5L = 0;
        TMR5H = 0;
        __delay_ms(1000);
        eusart_writeString(au8Ready);
        u16toa(TMR3L, au8Buffer, 10);
        eusart_writeString(au8Buffer);
        eusart_writeCharacter('-');
        u16toa(TMR5L, au8Buffer, 10);
        eusart_writeString(au8Buffer);
        
//        u16toa((uint16_t)((TMR1H<<8) + TMR1L), au8Buffer, 10);
//        eusart_writeString(au8Ready);
//        eusart_writeString(au8Buffer);
        //TMR1L = TMR1H = 0;
//        __delay_ms(10);
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
        u8Buffer = (uint8_t)(u16Data % u8Base);
        u16Data /= u8Base;
        if(u8Buffer >= 10)
            u8Buffer += 'A' - '0' - 10;
        u8Buffer += '0';
        *au8Buffer-- = u8Buffer;
    }
}