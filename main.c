/*
 * File:   main.c
 * Author: merayen
 *
 * Created on 2019/05/26, 16:43
 */

// CONFIG1
#pragma config FOSC = INTOSC    // Oscillator Selection (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config WDTE = OFF       // Watchdog Timer Enable (WDT disabled)
#pragma config PWRTE = OFF       // Power-up Timer Enable (PWRT enabled)
#pragma config MCLRE = ON       // MCLR Pin Function Select (MCLR/VPP pin function is MCLR)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Memory Code Protection (Data memory code protection is disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable (Brown-out Reset enabled)
#pragma config CLKOUTEN = ON   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = OFF        // Internal/External Switchover (Internal/External Switchover mode is enabled)
#pragma config FCMEN = OFF       // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is enabled)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config VCAPEN = OFF     // Voltage Regulator Capacitor Enable bit (Vcap functionality is disabled on RA6.)
#pragma config PLLEN = ON       // PLL Enable (4x PLL enabled)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LPBOR = OFF      // Low Power Brown-Out Reset Enable Bit (Low power brown-out is disabled)
#pragma config LVP = ON         // Low-Voltage Programming Enable (Low-voltage programming enabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>

/*
 * Port designations:
 * RA0/AN0          AN      Amp V+ voltage
 * RA1/AN1          AN      Amp V- voltage
 * RA2/AN2/DAC1     DAC     Fan speed heat sink 1
 * RA3/AN3          AN      ?
 * RA4              R       Power-button input
 * RA5/AN4/DAC2     DAC     Fan speed heat sink 2
 * RA6              R       ?
 * RA7              R       ?
 * RB0/AN12         AN      ?
 * RB1/AN10         AN      ?
 * RB2/AN8          DAC     Fan speed air intake
 * RB3/AN9          AN      ?
 * RB4/AN11         AN      ?
 * RB5/AN13         AN      ?
 * RB6              R       Mini powersupply power relay
 * RB7              R       Main powersupply power relay
 * RC0              R       Main powersupply live relay
 * RC1              R       Channel input power relays
 * RC2              R       Channel output power relays
 * RC3              R       Bridge enable output
 * RC4              R       Oscillation detected input (shuts off amp)
 * RC5              R       Offset detected input (shuts off amp)
 * RC6              R       Clip input Ch0
 * RC7              R       Clip input Ch1
 * RD0              R       Clip input Ch2
 * RD1/AN21         AN21    Heat sink temperature amp 1
 * RD2/DAC4         DAC     Fan speed air exhaust
 * RD3              R       Clip input Ch3
 * RD4              R       ?
 * RD5              R       Power LED Red
 * RD6              R       Power LED Green
 * RD7              R       Power LED Blue
 * RE0/AN5          AN      Heat sink amp 2
 * RE1/AN6          AN      Volume potensiometer ch0+1?
 * RE2/AN7          AN      Volume potensiometer ch2+3?
 */

#define TIMER_COUNT 10

#define SLOW_START_TIMER = 0
#define SLOW_START_TIMER_MS = 4000 // Wait 4 seconds until next stage in power-up



static long ticks = 0;
static long time = 0; // Milliseconds since boot
static long rounds = 0;


// State
static unsigned char main_state = 0;

struct Timer {
    unsigned int time;
    char started;
} timers[TIMER_COUNT];



void __interrupt () my_little_interrupt() {
    if (PIR1bits.TMR1IF) {
        ticks++;
        PIR1bits.TMR1IF = 0;
    }
}




void init_system(void) {
    // 32 MHz action
    OSCCONbits.SCS = 0;
    OSCCONbits.IRCF0 = 1;
    OSCCONbits.IRCF1 = 1;
    OSCCONbits.IRCF2 = 1;
    OSCCONbits.IRCF3 = 1;
    
    // Wait for the internal clock to be 32 MHz ready
    while (OSCSTATbits.HFIOFR == 0 || OSCSTATbits.HFIOFL == 0 || OSCSTATbits.HFIOFR == 0 || OSCSTATbits.PLLR == 1);
}




void init_time(void) {
    // TIMER1 configuration
    T1CONbits.TMR1CS = 0;
    T1CONbits.T1CKPS = 0;
    T1CONbits.TMR1ON = 1;

    // Reset TIMER1 state
    TMR1Lbits.TMR1L = 0;
    TMR1Hbits.TMR1H = 0;
    
    // Timer1 interrupt when rollover
    PIR1bits.TMR1IF = 0;
    PIE1bits.TMR1IE = 1;
    INTCONbits.PEIE = 1;
    INTCONbits.GIE = 1;
    
    for (int i = 0; i < TIMER_COUNT; i++) {
        timers[i].time = 0;
        timers[i].started = 0;
    }
}

void update_time(void) {
    time = ticks * 8; // This really gives crap time in milliseconds, but we don't need precision for the amp anyway
}




/*
 * Inits input buttons
 */
void init_buttons(void) {
    
}

void update_buttons(void) {
    
}




/*
 * Init input temperature readings
 */
void init_temperatures(void) {
    // TODO set up ports
}

void update_temperatures(void) {
    
}




/*
 * Clip functionality.
 * Makes volume react when clipping.
 */
void init_clip(void) {
    
}

void update_clip(void) {
    
}




/*
 * Volume control
 */
void init_volume(void) {
    
}

void update_volume(void) {
    
}




/*
 * Fan speed control
 */
void init_fans(void) {
    
}

void update_fans(void) {
    
}




/*
 * LEDs that lights up the amp owners life
 */
void init_leds(void) {
    
}

void update_leds(void) {
    
}




/*
 * Amplifier control
 */
void init_control(void) {
    
}

void update_control(void) {
    
}




void main(void) {
    init_system();
    init_time();
    init_temperatures();
    init_buttons();
    init_fans();
    init_leds();
    init_control();

    while (1) {
        update_time();
        update_temperatures();
        
        update_time();
        update_buttons();
        
        update_time();
        update_control();

        update_time();
        update_fans();
        
        update_time();
        update_leds();

        rounds++;
    }
}
