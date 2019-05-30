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
 * RA0/AN0          AN      in      Amp V+ voltage
 * RA1/AN1          AN      in      Amp V- voltage
 * RA2/AN2/DAC1     DAC
 * RA3/AN3          AN      ?
 * RA4              R       in      Power-button input
 * RA5/AN4/DAC2     DAC     out     Fan speed air intake
 * RA6              R       in      Enable bridge mode (2CO relay changing ch0+ch1 ground to ch2+ch3, and 2CO signal relay changes to 180 degree phase shift opamp)
 * RA7              R       out     Channel ch0+1 output power relay
 * RB0/AN12         AN      in      Heat sink temperature amp ch0
 * RB1/AN10         AN      in      Heat sink temperature amp ch1
 * RB2/AN8/DAC3     DAC     out     Fan speed heat sink
 * RB3/AN9          AN      in      Heat sink temperature amp ch2
 * RB4/AN11         AN      in      Heat sink temperature amp ch3
 * RB5/AN13         AN      ?
 * RB6              R       out     Mini powersupply power relay
 * RB7              R       out     Mini powersupply live relay
 * RC0              R       out     Main powersupply power relay
 * RC1              R       out     Main powersupply live relay
 * RC2              R       out     Channel ch2+3 output power relay
 * RC3              R       out     Bridge enable output
 * RC4              R       in      Oscillation detected input (shuts off amp)
 * RC5              R       in      Offset detected (shuts off amp)
 * RC6              R       in      Clip input Ch0+1
 * RC7              R       in      Clip input Ch2+3
 * RD0              R       out     PGA2310 (SDI)
 * RD1/AN21         AN      ?
 * RD2/DAC4         DAC     out     Fan speed air exhaust
 * RD3              R       out     PGA2310 (SCLK)
 * RD4              R       out     PGA2310 (CS)
 * RD5              R       out     Power LED Red
 * RD6              R       out     Power LED Green
 * RD7              R       out     Power LED Blue
 * RE0/AN5          AN      ?
 * RE1/AN6          AN      in      Volume potensiometer ch0+1?
 * RE2/AN7          AN      in      Volume potensiometer ch2+3?
 */

#define TIMER_COUNT 2

#define SLOW_START_TIMER 0
#define SLOW_START_TIMER_MS 4000 // Wait 4 seconds until next stage in power-up

#define TIMER_ADC_ACQUISITION 1
#define TIMER_ADC_ACQUISITION_MS 10 // 10 ms is overkill, but whatevs



static int ticks = 0; // How many ticks since last interrupt. Cleared everytime when read.
static long rounds = 0;


/* ---=== Main states ===--- */

// Amp is off. There is only power to this microcontroller via 9VA/9V-transformer
// Waits for user to push Power-button.
#define MAIN_STATE_STANDBY 0

// Power-on button har been pushed.
// Enables slow-start for low in-rush current for 160VA/12Vx2-transformer.
// Microcontroller monitors the speed of voltage rise, going directly to
// - MAIN_STATE_FAILURE if voltage doesn't raise fast enough (short-circuit, bad
// components etc)
// - MAIN_STATE_FAILURE if DC-offset or oscillation has been detected
// Will automatically go to MAIN_STATE_ON_LOW when voltage has gotten high enough.
#define MAIN_STATE_POWERON_LOW 1

// Amplifier is on and functional, using the 160VA/12Vx2-transformer.
// Output relays to the speakers are engaged.
// Goes to MAIN_STATE_POWERON_HIGH if one of the volume potensiometers exceeds
// a certain threshold as more voltage is needed.
#define MAIN_STATE_ON_LOW 2

// Amplifier is already on and powered by 160VA/12Vx2-transformer, but more
// power is needed.
// 630VA/24Vx2-transformer gets turned on and is now slow-starting for low
// in-rush current.
// Goes to state MAIN_STATE_ON_HIGH when voltage levels are high enough.
// Amplifier plays audio as usual in this mode, but volume is restricted until
// 630VA/24Vx2-transformer is ready and state MAIN_STATE_ON_HIGH has been
// reached.
#define MAIN_STATE_POWERON_HIGH 3

// All 3 transformers are now active and directly feeding current, though
// 160VA/12Vx2-transformer now only feeds the low-voltage pre-amplifier circuits
// as the 630VA/24Vx2-transformers dominates the voltage for the amp-stage.
#define MAIN_STATE_ON_HIGH 4

// Powers down.
// Sequential operations:
// - Volume is decreased quickly to -95dB (500ms)
// - Speakers gets disconnected
// - Wait 100ms
// - Slow-starter circuits gets engaged for both 160VA/12Vx2 and 630VA/24Vx2-transformers
// - Wait 100ms
// - Power is turned off for transformers 160VA/12Vx2 and 630VA/24Vx2
// - Goes to state MAIN_STATE_STANDBY
#define MAIN_STATE_POWER_DOWN 5

// Critical fault detected.
// Refuses to power on in this mode (power must be unplugged to trigger re-test).
// This happens when:
//  - Rail voltages are uneven (during power-up or when live)
//  - Too high rail voltages when using 160VA/12Vx2-transformer
//  - Too high rail voltages when using 630VA/24Vx2-transformer
//  - DC-offset has been detected on speaker output(s)
//  - Any oscillations detected on any speakers output(s)
//  - Clipping does not go away after several attempts at automatically reducing volume
//  - Too high temperatures on amp heat sink
//  - Too high ambient temperature (temperature sensor on microcontroller chip)
#define MAIN_STATE_FAILURE 10


/* Fans */
#define FAN_COUNT 2

#define FAN_INTAKE 0
#define FAN_EXHAUST 1


/* Volumes */
#define VOLUME_COUNT 4

#define VOLUME_LEFT_0 0
#define VOLUME_RIGHT_0 1
#define VOLUME_LEFT_1 2
#define VOLUME_RIGHT_1 3


/* Temperature sensors */
#define TEMP_COUNT 4

#define TEMP_AMBIENT 0
#define TEMP_AMP_HEAT_SINK_0 1
#define TEMP_AMP_HEAT_SINK_1 2


/* ADC inputs */
#define ADC_COUNT 9

#define ADC_AMP_POSITIVE_VOLTAGE 0 // +V rail to amp
#define ADC_AMP_NEGATIVE_VOLTAGE 1 // -V rail to amp
#define ADC_TEMPERATURE_HEAT_SINK_CH0 2
#define ADC_TEMPERATURE_HEAT_SINK_CH1 3
#define ADC_TEMPERATURE_HEAT_SINK_CH2 4
#define ADC_TEMPERATURE_HEAT_SINK_CH3 5
#define ADC_VOLUME_CH0_CH1 6
#define ADC_VOLUME_CH2_CH3 7
#define ADC_TEMPERATURE_AMBIENT 8




struct {
    unsigned char main          :4; // Main state of amp
    unsigned bridged            :1; // Set if amp is bridged
    unsigned power_button       :1; // If 1, power button is on
} state;


struct {
    struct {
        // Last value we accepted. If read value from potentiometer varies too much,
        // there might be an issue with the potensiometer.
        // Value is set to something low on power-up, so user needs to move the volume
        // down and then up again (safety against high volume)
        unsigned int read_accepted;
        
        // Targeted volume
        unsigned int target;
        
        // Actual volume that slowly increases to the target volume
        unsigned int actual;
    } volumes[VOLUME_COUNT];

    struct {
        unsigned char speed; // The speed set on to the fan
    } fan_speed[FAN_COUNT];

} values;


struct {
    unsigned int temperature; // Format: Celcius * 100
    unsigned int slow_temperature; // Used to calculate how fast temperature rises/falls. Needed? Maybe not?
} temperatures[TEMP_COUNT];


struct {
    unsigned int time; // Current time
    unsigned int goal; // Time when the timer stops and is "finished"
} timers[TIMER_COUNT];


struct {
    unsigned char current; // Current ADC that is being measured. -1 means that no ADC conversion has taken place yet
    unsigned char state; // See update_adc() on how it is used
    struct {
        unsigned int value;
        unsigned chs : 5; // The CHS-register id, set by init_adc()
    } entries[ADC_COUNT];
} adc;




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
    
    OPTION_REGbits.nWPUEN = 1; // Disable weak pull-ups globally. Easier to configure ADC. For now.
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
        timers[i].goal = 0;
    }
}

void update_time(void) {
    int time = ticks * 8; // This really gives crap time, in milliseconds, but we don't need precision for the amp anyway
    ticks = 0; // May loose a tick if interrupted around this part, but whatever
    for (int i = 0; i < TIMER_COUNT; i++) {
        if (timers[i].time < timers[i].goal)
            timers[i].time += time;
    }
}

/*
 * Start a timer.
 */
void timer_start(unsigned char timer, unsigned int goal) {
    timers[timer].time = 0;
    timers[timer].goal = goal;
}

/*
 * Check if timer has reached its goal.
 * Returns 0 if not, 1 if yes.
 */
char timer_finished(unsigned char timer) {
    return timers[timer].time >= timers[timer].goal;
}




/*
 * Inits input buttons
 */
void init_buttons(void) {
    TRISAbits.TRISA4 = 1;
    TRISAbits.TRISA6 = 1;
}

void update_buttons(void) {
    // TODO implement some delay, like 10ms
    if (PORTAbits.RA4) { // Power-button is on
        state.power_button = 1;
    } else {
        state.power_button = 0;
    }
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
 Rail voltage monitoring
 */
void init_voltage(void) {
    
}

void update_voltage(void) {
    
}




/*
 * Clip functionality.
 * Makes volume react when clipping.
 */
void init_clip(void) {
    TRISCbits.TRISC6 = 1;
    TRISCbits.TRISC7 = 1;
}

void update_clip(void) {
    // TODO Lower volume by 3 dB if any clipping, every 100ms
}




/*
 * Volume control
 */
void init_volume(void) {
    TRISEbits.TRISE1 = 0;
    TRISEbits.TRISE1 = 0;
}

void update_volume(void) {
    
}




/*
 * Speaker control (relays)
 */
void init_speakers(void) {
    
}

void update_speakers(void) {
    
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
    TRISDbits.TRISD5 = 0; // Red
    TRISDbits.TRISD6 = 0; // Green
    TRISDbits.TRISD7 = 0; // Blue
    PORTDbits.RD5 = 0;
    PORTDbits.RD6 = 0;
    PORTDbits.RD7 = 0;
}

void update_leds(void) {
    if (state.main == MAIN_STATE_STANDBY) {
        PORTDbits.RD5 = 0;
        PORTDbits.RD6 = 0;
        PORTDbits.RD7 = 0;
    } else if (state.main == MAIN_STATE_POWERON_LOW) {
        PORTDbits.RD5 = 1;
        PORTDbits.RD6 = 1;
        PORTDbits.RD7 = 0;
    } else if (state.main == MAIN_STATE_ON_LOW) {
        PORTDbits.RD5 = 0;
        PORTDbits.RD6 = 1;
        PORTDbits.RD7 = 0;
    } else if (state.main == MAIN_STATE_POWERON_HIGH) {
        PORTDbits.RD5 = 1;
        PORTDbits.RD6 = 1;
        PORTDbits.RD7 = 0;
    } else if (state.main == MAIN_STATE_ON_HIGH) {
        PORTDbits.RD5 = 0;
        PORTDbits.RD6 = 0;
        PORTDbits.RD7 = 1;
    } else if (state.main == MAIN_STATE_POWER_DOWN) {
        PORTDbits.RD5 = 1;
        PORTDbits.RD6 = 1;
        PORTDbits.RD7 = 1;
    } else if (state.main == MAIN_STATE_FAILURE) {
        PORTDbits.RD5 = 1;
        PORTDbits.RD6 = 0;
        PORTDbits.RD7 = 0;
    }
}


/*
 * Does all the ADC stuff.
 * Does measuring all the time, async.
 */
void init_adc(void) {
    ANSELAbits.ANSA0 = 1;
    TRISAbits.TRISA0 = 1;
    adc.entries[ADC_AMP_POSITIVE_VOLTAGE].chs = 0;

    ANSELAbits.ANSA1 = 1;
    TRISAbits.TRISA1 = 1;
    adc.entries[ADC_AMP_NEGATIVE_VOLTAGE].chs = 0b00001;

    ANSELBbits.ANSB0 = 1;
    TRISBbits.TRISB0 = 1;
    adc.entries[ADC_TEMPERATURE_HEAT_SINK_CH0].chs = 0b01100;

    ANSELBbits.ANSB1 = 1;
    TRISBbits.TRISB1 = 1;
    adc.entries[ADC_TEMPERATURE_HEAT_SINK_CH1].chs = 0b01010;

    ANSELBbits.ANSB3 = 1;
    TRISBbits.TRISB3 = 1;
    adc.entries[ADC_TEMPERATURE_HEAT_SINK_CH2].chs = 0b01001;

    ANSELBbits.ANSB4 = 1;
    TRISBbits.TRISB4 = 1;
    adc.entries[ADC_TEMPERATURE_HEAT_SINK_CH3].chs = 0b01011;

    ANSELEbits.ANSE1 = 1;
    TRISEbits.TRISE1 = 1;
    adc.entries[ADC_VOLUME_CH0_CH1].chs = 0b00110;

    ANSELEbits.ANSE2 = 1;
    TRISEbits.TRISE2 = 1;
    adc.entries[ADC_VOLUME_CH2_CH3].chs = 0b00111;

    // Internal PIC tempADC_VOLUME_CH2_CH3
    adc.entries[ADC_TEMPERATURE_AMBIENT].chs = 0b11101;


    ADCON1bits.ADCS = 2; // Fosc/32
    ADCON1bits.ADPREF = 0; // +Vref for ADC from Vdd
    ADCON1bits.ADNREF = 0; // - Vref for ADC from Ground

    ADCON0bits.ADON = 1;

    adc.current = 0;

    for (int i = 0; i < ADC_COUNT; i++)
        adc.entries[i].value = 0;
}

void update_adc(void) {
    if (adc.state == 0) { // Doing nothing
        // Select the correct ADC-input
        ADCON0bits.CHS = adc.entries[adc.current].chs;
        
        // Start the timer, as we need to wait a period of time to start the ADC-conversion (charging hold-capacitor))
        timer_start(TIMER_ADC_ACQUISITION, TIMER_ADC_ACQUISITION_MS);

        adc.state = 1;

    }

    if (adc.state == 1) { // Waiting on acquisition time to meet its goal
        if (timer_finished(TIMER_ADC_ACQUISITION)) {
            ADCON0bits.GO = 1; // Start conversion
            adc.state = 2;
        }

    }

    if (adc.state == 2) {
        if (ADCON0bits.GO == 0) {
            adc.entries[adc.current].value = (ADRESHbits.ADRESH << 8) + ADRESLbits.ADRESL; // FIXME Probably wrong. Probably...

            // Select the next ADC-input to measure
            adc.current++;
            adc.current %= ADC_COUNT;

            // Go back to beginning to begin measuring again
            adc.state = 0;
        }
    }
}




/*
 * Amplifier control
 */
void init_control(void) {
    state.main = MAIN_STATE_STANDBY;
    state.bridged = 0;
}

void update_control(void) {
    if (state.main == MAIN_STATE_STANDBY) {
        if (state.power_button) {
            // Power button is on. We need to boot up
            state.main = MAIN_STATE_POWERON_LOW;
        }
    } else if (state.main == MAIN_STATE_POWERON_LOW) {
        // Amplifier is warming up with the 160VA/12Vx2-transformer
        
    } else if (state.main == MAIN_STATE_ON_LOW) {
        
    } else if (state.main == MAIN_STATE_POWERON_HIGH) {
        
    } else if (state.main == MAIN_STATE_ON_HIGH) {
        
    } else if (state.main == MAIN_STATE_POWER_DOWN) {
        
    } else if (state.main == MAIN_STATE_FAILURE) {
        
    }
}




void main(void) {
    init_system();
    init_time();
    init_temperatures();
    init_buttons();
    init_fans();
    init_leds();
    init_voltage();
    init_clip();
    init_volume();
    init_speakers();
    init_adc();
    init_control();

    while (1) {
        update_time();
        update_adc();
        
        update_time();
        update_temperatures();

        update_time();
        update_buttons();

        update_time();
        update_fans();

        update_time();
        update_leds();

        update_time();
        update_voltage();
        
        update_time();
        update_clip();
        
        update_time();
        update_volume();
        
        update_time();
        update_speakers();

        update_time();
        update_control();

        rounds++;
    }
}
