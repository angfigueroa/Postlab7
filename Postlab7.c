/* 
 * File:   P_7.c
 * Author: ANGELA FIGUEROA
 *
 * Created on 14 de abril de 2023, 11:58
 */


// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)
// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

#include <xc.h>
#include <stdint.h>
#include <stdio.h>

//definicion de frecuencia para delay
#define _XTAL_FREQ 4000000

//definiciones generales
#define tmr0_val 249

//definiciones generales
#define POT_MIN 0
#define POT_MAX 255
#define PWM_MIN 100
#define PWM_MAX 650

//definicion de alias para pines
#define fake_pwm PORTCbits.RC3

//variables globales
unsigned short CCPR = 0;
unsigned short CCPR_b = 0;
uint8_t cont_led;
int pwm_led;

//funciones
void setup(void);

void tmr0_setup(void);

//funcion para interpolacion
unsigned short interpol(uint8_t val, uint8_t pot_min, uint8_t pot_max,
        unsigned short pwm_min, unsigned short pwm_max);

//config principal
void setup(void){
    ANSEL = 0b00000111;
    ANSELH = 0;
    
    TRISA = 0b00000111;
    PORTA = 0;
    PORTC = 0;
    
    //config de interrupciones
    INTCONbits.GIE = 1;
    INTCONbits.PEIE = 1;
    INTCONbits.T0IE = 1;
    PIR1bits.ADIF = 0;
    PIE1bits.ADIE = 1;
    INTCONbits.T0IF = 0;
    //OSCCON
    OSCCONbits.IRCF = 0b0110;
    OSCCONbits.SCS = 1;
    
    //config ADC
    ADCON0bits.ADCS = 0b10;
    ADCON1bits.VCFG0 = 0;
    ADCON1bits.VCFG1 = 0;
    
    ADCON0bits.CHS = 0b0000;
    ADCON1bits.ADFM = 0;
    ADCON0bits.ADON = 1;
    __delay_us(320);
    
    //config PWM
    TRISCbits.TRISC2 = 1;
    TRISCbits.TRISC1 = 1;
    PR2 = 249;
    
    //config CCP
    CCP1CON = 0;
    CCP2CON = 0;
    CCP1CONbits.P1M = 0;
    CCP1CONbits.CCP1M = 0b1100;
    CCP2CONbits.CCP2M = 0b1100;
    
    CCPR1L = 250>>2;
    CCP1CONbits.DC1B = 250 & 0b11;
    CCPR2L = 250>>2;
    CCP2CONbits.DC2B0 = 250 & 0b01;
    CCP2CONbits.DC2B1 = 250 & 0b10;
    
    T2CONbits.T2CKPS = 0b11;
    PIR1bits.TMR2IF = 0;
    T2CONbits.TMR2ON = 1;
    while(!PIR1bits.TMR2IF);
    PIR1bits.TMR2IF = 0;
    
    TRISCbits.TRISC3 = 0;
    TRISCbits.TRISC2 = 0;
    TRISCbits.TRISC1 = 0;
    return;
}

//config TMR0
void tmr0_setup(void){
    OPTION_REGbits.T0CS = 0;
    OPTION_REGbits.PSA = 0;
    OPTION_REGbits.PS0 = 0;
    OPTION_REGbits.PS1 = 0;
    OPTION_REGbits.PS2 = 0;
    
    INTCONbits.T0IF = 0;
    TMR0 = tmr0_val;
    return;
}

//interpolacion de valores
unsigned short interpol(uint8_t val, uint8_t pot_min, uint8_t pot_max,
        unsigned short pwm_min, unsigned short pwm_max){
    
    return (unsigned short)(pwm_min+((float)(pwm_max-pwm_min)/(pot_max-pot_min))
            *(val-pot_min));
}

//interrupciones
void __interrupt() isr(void){
    if (PIR1bits.ADIF){
        if (ADCON0bits.CHS == 0){
            CCPR = interpol(ADRESH, POT_MIN, POT_MAX, PWM_MIN, PWM_MAX);
            CCPR1L = (uint8_t)(CCPR>>2);
            CCP1CONbits.DC1B = CCPR & 0b11;    
        }
        else if (ADCON0bits.CHS == 1){
            CCPR_b = interpol(ADRESH, POT_MIN, POT_MAX, PWM_MIN, PWM_MAX);
            CCPR2L = (uint8_t)(CCPR_b>>2);
            CCP2CONbits.DC2B0 = CCPR_b & 0b01; 
            CCP2CONbits.DC2B1 = CCPR_b & 0b10;     
        }
        
        else if (ADCON0bits.CHS == 2){
            pwm_led = ADRESH;
        }
        PIR1bits.ADIF = 0;
    }
    if(INTCONbits.T0IF){
        cont_led++;
        
        if (cont_led <= pwm_led){
            fake_pwm = 1;// si el valor del contador es menor o igual, encender el puerto
        }
        else{
            fake_pwm = 0;
        }
        INTCONbits.T0IF = 0;
        TMR0 = tmr0_val;
    }
}

void main(void){
    setup();
    tmr0_setup();
    
    //loop principal
    while(1){
        if (ADCON0bits.GO == 0){ //revisar si el ADC esta encendido 
            if (ADCON0bits.CHS == 0){//revisar si se encuantra en canal anlogico 0 
                ADCON0bits.CHS = 1;// Cambio a canal analogico 1
                __delay_us(40);//Tiempo  estabelizado
            }
            else if (ADCON0bits.CHS == 1){//revisar si se encuantra en canal anlogico 1
                ADCON0bits.CHS = 2;// Cambio a canal analogico 2
                __delay_us(40);//Tiempo  estabelizado
            }
            else if (ADCON0bits.CHS == 2){//revisar si se encuantra en canal anlogico 2
                ADCON0bits.CHS = 0;// Cambio a canal analogico 0
                __delay_us(40);//Tiempo  estabelizado
            }
            __delay_us(40);//Tiempo  estabelizado
            ADCON0bits.GO = 1;
        }    
                 
    }
    return;
}
