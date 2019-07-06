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
#include <math.h>

/*
 * Port designations:
 * RA0/AN0          AN      in      Amp V+ voltage
 * RA1/AN1          AN      in      Amp V- voltage
 * RA2/AN2/DAC1     DAC
 * RA3/AN3          AN      ?
 * RA4              R       in      Power-button input
 * RA5/AN4/DAC2     DAC     out     Fan speed heat sinks Ch0+Ch1
 * RA6              R       in      Enable bridge mode (2CO relay changing ch0+ch1 ground to ch2+ch3, and 2CO signal relay changes to 180 degree phase shift opamp)
 * RA7              R       out     Bridge enable output (Ch2+Ch3 switch over to ground on Ch0 and Ch1, and 2CO signal relay for 180 phase shift via op-amp)
 * RB0/AN12         AN      in      Heat sink temperature amp ch0
 * RB1/AN10         AN      in      Heat sink temperature amp ch1
 * RB2/AN8/DAC3     DAC     out     Fan speed heat sinks Ch2+Ch3
 * RB3/AN9          AN      in      Heat sink temperature amp ch2
 * RB4/AN11         AN      in      Heat sink temperature amp ch3
 * RB5/AN13         AN      ?
 * RB6              R       out     Mini powersupply power relay
 * RB7              R       out     Mini powersupply live relay
 * RC0              R       out     Main powersupply power relay
 * RC1              R       out     Main powersupply live relay
 * RC2              R       out     Channel ch0+1 output power relay
 * RC3              R       out     Channel ch2+3 output power relay
 * RC4              R       in      Oscillation detected input (shuts off amp)
 * RC5              R       in      Offset detected (shuts off amp)
 * RC6              R       in      Clip input Ch0+1
 * RC7              R       in      Clip input Ch2+3
 * RD0              R       out     PGA2310 (SDI)
 * RD1/AN21         AN      ?
 * RD2/DAC4         DAC     ?
 * RD3              R       out     PGA2310 (SCLK)
 * RD4              R       out     PGA2310 (CS)
 * RD5              R       out     Power LED Red
 * RD6              R       out     Power LED Green
 * RD7              R       out     Power LED Blue
 * RE0/AN5          AN      ?
 * RE1/AN6          AN      in      Volume potensiometer ch0+1?
 * RE2/AN7          AN      in      Volume potensiometer ch2+3?
 */




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



/* Timers */
#define TIMER_COUNT 4

#define MAIN_STATE_CHANGE_TIMER 0 // Timer used when changing states
#define TIMER_ADC_ACQUISITION 1
#define TIMER_CLIP_CH0_CH1 2
#define TIMER_CLIP_CH2_CH3 3




/* Fans */
#define FAN_COUNT 2

#define FAN_INTAKE 0
#define FAN_EXHAUST 1


/* Volumes */
#define VOLUME_LANE_COUNT 2

#define VOLUME_CH0_CH1 0
#define VOLUME_CH2_CH3 1


/* Temperature sensors */
#define TEMP_COUNT 3

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


// Failure Codes
#define FAILURE_CLIP 1 // Amplifier clips even though volume is low (may be internal error)
#define FAILURE_RAIL_VOLTAGE 2 // Un-even or too low rail-voltage (under boot-up or "normal" operation)
#define FAILURE_TEMPERATURE 4 // Too high temperature somewhere
#define FAILURE_DC_OFFSET 8 // DC-offset has been detected on one of the speaker outputs
#define FAILURE_OSCILLATION 16// Oscillation has been detected on one of the speaker outputs



/* General configuration*/
#define VOLUME_HYSTERESIS 1 // PIC won't change the volume if the potensiometer is inside this range (0 - 255)
#define VOLUME_MAX_DIFFERENCE 8 // PIC won't change the volume if potensiometer suddenly varies more than this (0 - 255)
#define VOLUME_LIMIT_LOW 184 // Absolute max volume that will be sent to the PGA2310s with 160VA/12Vx2 transformer. Formula in Python: (lambda Vsignal, gain, Vout: (log(Vout/(Vsignal*gain)) / log(10) + 9.6) * 20)(1, 1, 10**3.15) --> 255
#define VOLUME_LIMIT_HIGH 212 // Absolute max volume that will be sent to the PGA2310s with 630VA/24VAx2 transformer.
#define CLIP_MINIMAL_VOLUME_BEFORE_SHUTOFF 180 // 180 == -6dB. If we still clip here, something is wrong with the amp. Shut off
#define CLIP_VOLUME_REDUCTION_WAIT 100 // How many ms to wait until next reduction of volume due to clipping
#define TIMER_ADC_ACQUISITION_MS 10 // 10 ms is overkill, but whatevs
#define SLOW_START_LOW_TIMER_MS 3000 // Time waiting from enabling 160VA/12Vx2-transformer to make it live
#define SLOW_START_HIGH_TIMER_MS 2000 // Time waiting from enabling 630VA/24Vx2-transformer to make it live


struct {
    unsigned main               :4; // Main state of amp
    unsigned failure            :6; // If any failure (see Failure Codes)
    unsigned bridged            :1; // Set if amp should be bridged
    unsigned power_button       :1; // If 1, power button is on
    struct {
        unsigned requiring_high  :1; // High power is required (volume is above VOLUME_LIMIT_LOW)
        unsigned low_stable     :1; // Power from mini transformer is stable and ready 
        unsigned high_stable    :1; // Power from main transformer is stable and ready
    } power;
} state;


struct {
    struct {
        unsigned char speed; // The speed set on to the fan
    } fan_speed[FAN_COUNT];

} values;


struct {
    struct {
        // Targeted volume. Can be set anytime. Will automatically get sent to PGA2310s upon change
        unsigned char target;

        // Maximum volume allowed. Is set to something below 255 if any clipping has occured.
        // Slowly raises again.
        unsigned char clip_limit;

        // Maximum volume allowed due to temperature. Is 255 when everything is inside allowed temperature range.
        // Slowly raises back to 255 if temperature gets into allowed range again.
        unsigned char temperature_limit;

        // Maximum volume allowed due to power (as we won't allow full volume when using the 160VA/12Vx2 transformer)
        unsigned char power_limit;

        // Actual volume that has been successfully sent to the PGA2310s.
        // Read this one to get current volume level. Never set.
        // This is the result of: min(target, min(clip_limit, temperature_limit))
        unsigned char actual;
    } lanes[VOLUME_LANE_COUNT];
} volume;


struct {
    struct {
        int temperature; // Format: Celcius * 100
    } entries[TEMP_COUNT];
    //unsigned int slow_temperature; // Used to calculate how fast temperature rises/falls. Needed? Maybe not?
} temperature;


struct {
    unsigned int time; // Current time, in ms
    unsigned int goal; // Time when the timer stops and is "finished"
} timers[TIMER_COUNT];


struct {
    unsigned char current; // Current ADC that is being measured
    unsigned char state; // See update_adc() on how it is used
    struct {
        unsigned int value;
        unsigned chs : 5; // The CHS-register id, set by init_adc()
    } entries[ADC_COUNT];
} adc;




void __interrupt () my_little_interrupting_pony() {
    if (PIR1bits.TMR1IF) {
        ticks++;
        PIR1bits.TMR1IF = 0;
    }
}




void system_init(void) {
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




void time_init(void) {
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

void time_update(void) {
    int time = ticks * 8; // This really gives crap time, in milliseconds, but we don't need precision for the amp anyway
    ticks = 0; // May loose a tick if interrupted around this part, but whatever
    for (int i = 0; i < TIMER_COUNT; i++)
        if (timers[i].time < timers[i].goal)
            timers[i].time += time;
}

/*
 * Start a timer with a goal time (in ms)
 */
void timer_start(unsigned char timer, unsigned int goal) {
    timers[timer].time = 0;
    timers[timer].goal = goal;
}

void timer_clear(unsigned char timer) {
    timers[timer].goal = 0;
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
void buttons_init(void) {
    TRISAbits.TRISA4 = 1;
    TRISAbits.TRISA6 = 1;
    ANSELAbits.ANSA4 = 0;
}

void buttons_update(void) {
    // TODO implement some delay, like 10ms
    state.power_button = PORTAbits.RA4;
    state.bridged = PORTAbits.RA6;
}




/*
 * Init input temperature readings
 */
void temperature_init(void) {
    
}

int temperature_calculate(unsigned char temp_no) {
    return adc.entries[temp_no].value; // TODO Do the math. We do hopefully have the same NTC-resistor for all temperature measurements
}

void temperature_update(void) {
    temperature_calculate(ADC_TEMPERATURE_HEAT_SINK_CH0);
    temperature_calculate(ADC_TEMPERATURE_HEAT_SINK_CH1);
    temperature_calculate(ADC_TEMPERATURE_HEAT_SINK_CH2);
    temperature_calculate(ADC_TEMPERATURE_HEAT_SINK_CH3);
    temperature_calculate(ADC_TEMPERATURE_AMBIENT);
}




/*
 Rail voltage monitoring
 */
void init_voltage(void) {
    ANSELCbits.ANSC0 = 0;
    TRISCbits.TRISC0 = 1;

    ANSELCbits.ANSC1 = 0;
    TRISCbits.TRISC1 = 1;
}

void update_voltage(void) {
    
}




/*
 * Clip functionality.
 * Makes volume react when clipping.
 */
void init_clip(void) {
    ANSELCbits.ANSC6 = 0;
    TRISCbits.TRISC6 = 1;

    ANSELCbits.ANSC7 = 0;    
    TRISCbits.TRISC7 = 1;
}

void update_clip(void) {
    if (state.failure & FAILURE_CLIP)
        return; // We have already flagged that the amp should be turned off

    if (state.main != MAIN_STATE_ON_LOW && state.main != MAIN_STATE_POWERON_HIGH && state.main != MAIN_STATE_ON_HIGH)
        return; // We are disabled outside these modes

    if (PORTDbits.RD6 && timer_finished(TIMER_CLIP_CH0_CH1)) { // Ch0/Ch1 is clipping. Reduce volume on this lane
        if (volume.lanes[VOLUME_CH0_CH1].actual < CLIP_MINIMAL_VOLUME_BEFORE_SHUTOFF) {
            state.failure |= FAILURE_CLIP; // Volume is too low to actually clip, something else is wrong!
        } else {
            volume.lanes[VOLUME_CH0_CH1].clip_limit = volume.lanes[VOLUME_CH0_CH1].actual - 6; // Turn down volume by 3dB
            timer_start(TIMER_CLIP_CH0_CH1, CLIP_VOLUME_REDUCTION_WAIT); // Wait for a little while to see if the clipping goes away
        }
    }

    if (PORTDbits.RD7 && timer_finished(TIMER_CLIP_CH2_CH3)) { // Ch2/Ch3 is clipping. Reduce volume on this lane
        if (volume.lanes[VOLUME_CH2_CH3].actual < CLIP_MINIMAL_VOLUME_BEFORE_SHUTOFF) {
            state.failure |= FAILURE_CLIP; // Volume is too low to actually clip, something else is wrong!
        } else {
            volume.lanes[VOLUME_CH2_CH3].clip_limit = volume.lanes[VOLUME_CH2_CH3].actual - 6; // Turn down volume by 3dB
            timer_start(TIMER_CLIP_CH2_CH3, CLIP_VOLUME_REDUCTION_WAIT); // Wait for a little while to see if the clipping goes away
        }
    }
}




/*
 * Volume control.
 */
void init_volume(void) {
    TRISDbits.TRISD0 = 0;
    TRISDbits.TRISD3 = 0;
    TRISDbits.TRISD4 = 0;
    PORTDbits.RD0 = 0;
    PORTDbits.RD3 = 0;
    PORTDbits.RD4 = 1; // "Disables" the data flow to the PGA2310s

    for (int i = 0; i < VOLUME_LANE_COUNT; i++) {
        volume.lanes[i].actual = 0;
        volume.lanes[i].target = 0; // TODO read this value from the volume potensiometers on boot-up? User has to bring the potensiometers down and then up again to have any audio
        volume.lanes[i].clip_limit = 255; // Init as "no limit"
        volume.lanes[i].temperature_limit = 255; // Init as "no limit"
        volume.lanes[i].power_limit = 0; // Init as "no volume allowed". This limit will be lifted when amp has powered up
    }
}

void update_from_volume_potentiometers(unsigned char adc_no, unsigned char lane_no) {
    unsigned int next_volume = adc.entries[adc_no].value / 32; // From ADC 0-8191 to volume in 0-255, as represented in the PGA2310s

    // Some local safety
    if (next_volume > 255)
        next_volume = 255;

    int diff = abs(next_volume - volume.lanes[lane_no].actual);

    if (diff >= VOLUME_HYSTERESIS && diff <= VOLUME_MAX_DIFFERENCE)
        volume.lanes[lane_no].target = next_volume;
}

// Limits volume by different limits
void update_volume_by_limits(void) {
    for (unsigned char lane = 0; lane < VOLUME_LANE_COUNT; lane++) {
        unsigned char target = volume.lanes[lane].target;
        if (target > volume.lanes[lane].clip_limit)
            target = volume.lanes[lane].clip_limit;

        if (target > volume.lanes[lane].temperature_limit)
            target = volume.lanes[lane].temperature_limit;

        if (target > volume.lanes[lane].power_limit)
            target = volume.lanes[lane].power_limit;

        volume.lanes[lane].target = target;
    }
}

void update_volume_pga2310(void) {
    // See if any of the volume variables has been changed
    char has_changed = 0;
    for (int i = 0; i < VOLUME_LANE_COUNT; i++)
        if (volume.lanes[i].actual != volume.lanes[i].target)
            has_changed = 1; // There is a change

    if (has_changed == 0)
        return; // Nothing to do

    PORTDbits.RD3 = 0;
    PORTDbits.RD4 = 0; // "Enables" the PGA2310s to receive data

    // Sends all the data directly. PIC is at 32MHz, 8M instructions per second, while PGA2310 receive data at 6.25MHz
    // Hopefully works as long as the C-compiler doesn't optimize too much
    for (int lane = 0; lane < VOLUME_LANE_COUNT; lane++) {
        for (char channel = 0; channel < 2; channel++) {
            for (char position = 0; position < 8; position++) {
                PORTDbits.RD3 = 0;
                PORTDbits.RD0 = (volume.lanes[lane].target >> (7 - position)) % 2;
                PORTDbits.RD3 = 1;
            }
        }
    }

    PORTDbits.RD4 = 1; // Disables the PGA2310s to receive any data
    PORTDbits.RD3 = 0;
}

void update_volume(void) {
    if (state.main == MAIN_STATE_STANDBY || state.main == MAIN_STATE_POWERON_LOW || state.main == MAIN_STATE_FAILURE) {
        for (int lane = 0; lane < VOLUME_LANE_COUNT; lane++)
            volume.lanes[lane].power_limit = 0; // If in any of these states above, force volume to mute

    } else { // Normal operation state. Read volume potensiometers
        update_from_volume_potentiometers(ADC_VOLUME_CH0_CH1, VOLUME_CH0_CH1);
        update_from_volume_potentiometers(ADC_VOLUME_CH2_CH3, VOLUME_CH2_CH3);

        if (state.main == MAIN_STATE_ON_LOW || state.main == MAIN_STATE_POWERON_HIGH) {
            // Limit volume available when using 160VA/12Vx2 transformer and when slow-starting to the 630VA/24Vx2 transformer
            for (int lane = 0; lane < VOLUME_LANE_COUNT; lane++)
                volume.lanes[lane].power_limit = VOLUME_LIMIT_LOW;
        } else if (state.main == MAIN_STATE_ON_HIGH) {
            for (int lane = 0; lane < VOLUME_LANE_COUNT; lane++)
                volume.lanes[lane].power_limit = VOLUME_LIMIT_HIGH;
        }
    }

    update_volume_by_limits();
    update_volume_pga2310();
}




/*
 * Speaker control (relays)
 */
void init_relays(void) {
    TRISBbits.TRISB6 = 0; // Mini transformer
    LATBbits.LATB6 = 0;

    TRISBbits.TRISB7 = 0; // Mini transformer live
    LATBbits.LATB7 = 0;

    TRISCbits.TRISC0 = 0; // Main transformer
    LATCbits.LATC0 = 0;

    TRISCbits.TRISC1 = 0; // Main transformer
    LATCbits.LATC1 = 0;

    TRISCbits.TRISC2 = 0; // Channel Ch0+Ch1
    LATCbits.LATC2 = 0;

    TRISCbits.TRISC3 = 0; // Channel Ch2+Ch3
    LATCbits.LATC3 = 0;

    TRISAbits.TRISA7 = 0; // Bridge enable
    LATAbits.LATA7 = 0;
}

void update_relays(void) {
    // Powering
    if (state.main == MAIN_STATE_STANDBY || state.main == MAIN_STATE_FAILURE) {
        // This is probably bad.
        // Should wait a bit until all speakers are disconnected before doing this one (e.g 180 phase shift to 0 phase shift if bridge relays are quicker than the output relays)
        LATBbits.LATB6 = 0;
        LATBbits.LATB7 = 0;
        LATCbits.LATC0 = 0;
        LATCbits.LATC1 = 0;

    } else if (state.main == MAIN_STATE_POWERON_LOW) {
        LATBbits.LATB6 = 1;
        LATBbits.LATB7 = 0;
        LATCbits.LATC0 = 0;
        LATCbits.LATC1 = 0;

    } else if (state.main == MAIN_STATE_ON_LOW) {
        LATBbits.LATB6 = 1;
        LATBbits.LATB7 = 1;
        LATCbits.LATC0 = 0;
        LATCbits.LATC1 = 0;

    } else if (state.main == MAIN_STATE_POWERON_HIGH) {
        LATBbits.LATB6 = 1;
        LATBbits.LATB7 = 1;
        LATCbits.LATC0 = 1;
        LATCbits.LATC1 = 0;

    } else if (state.main == MAIN_STATE_ON_HIGH) {
        LATBbits.LATB6 = 1;
        LATBbits.LATB7 = 1;
        LATCbits.LATC0 = 1;
        LATCbits.LATC1 = 1;

    } else if (state.main == MAIN_STATE_POWER_DOWN) {
        LATBbits.LATB6 = 1;
        LATBbits.LATB7 = 0;
        LATCbits.LATC0 = 1;
        LATCbits.LATC1 = 0;
    }


    // Speaker outputs
    if (
        state.main == MAIN_STATE_STANDBY ||
        state.main == MAIN_STATE_FAILURE ||
        state.main == MAIN_STATE_POWERON_LOW
    ) {
        LATCbits.LATC2 = 0;
        LATCbits.LATC3 = 0;

    } else if (
        state.main == MAIN_STATE_ON_LOW ||
        state.main == MAIN_STATE_POWERON_HIGH ||
        state.main == MAIN_STATE_ON_HIGH
    ) {
        LATCbits.LATC2 = 1;
        // LATC3 gets updated under
    }


    // Bridging
    if (state.main == MAIN_STATE_POWERON_LOW) {
        // Only update bridge mode under powering-up
        if (state.bridged) {
            LATCbits.LATC3 = 0;
            LATAbits.LATA7 = 1;
        } else {
            LATAbits.LATA7 = 0;
        }
    } else if (state.main == MAIN_STATE_STANDBY) {
        LATAbits.LATA7 = 0;
    }
}




/*
 * Fan speed control
 */
void init_fans(void) {
    DAC2CON0bits.DAC2PSS = 00; // Vdd
    DAC2CON0bits.DAC2OE1 = 1;
    DAC2CON0bits.DAC2OE2 = 0;
    DAC2CON0bits.DAC2EN = 1;

    DAC3CON0bits.DAC3PSS = 00; // Vdd
    DAC3CON0bits.DAC3OE1 = 1;
    DAC3CON0bits.DAC3OE2 = 0;
    DAC3CON0bits.DAC3EN = 1;
}

void update_fans(void) {
    DAC2CON1bits.DAC2R = 24; // 75% for now, for testing
    DAC3CON1bits.DAC3R = 8; // 25% for now, for testing
}




/*
 * LEDs that lights up the amp owners life
 */
void leds_init(void) {
    TRISDbits.TRISD5 = 0; // Red
    TRISDbits.TRISD6 = 0; // Green
    TRISDbits.TRISD7 = 0; // Blue
    LATDbits.LATD5 = 0;
    LATDbits.LATD6 = 0;
    LATDbits.LATD7 = 0;
}

void leds_update(void) {
    if (state.main == MAIN_STATE_STANDBY) {
        LATDbits.LATD5 = 0;
        LATDbits.LATD6 = 0;
        LATDbits.LATD7 = 0;
    } else if (state.main == MAIN_STATE_POWERON_LOW) {
        LATDbits.LATD5 = 1;
        LATDbits.LATD6 = 1;
        LATDbits.LATD7 = 0;
    } else if (state.main == MAIN_STATE_ON_LOW) {
        LATDbits.LATD5 = 0;
        LATDbits.LATD6 = 1;
        LATDbits.LATD7 = 0;
    } else if (state.main == MAIN_STATE_POWERON_HIGH) {
        LATDbits.LATD5 = 1;
        LATDbits.LATD6 = 1;
        LATDbits.LATD7 = 0;
    } else if (state.main == MAIN_STATE_ON_HIGH) {
        LATDbits.LATD5 = 0;
        LATDbits.LATD6 = 0;
        LATDbits.LATD7 = 1;
    } else if (state.main == MAIN_STATE_POWER_DOWN) {
        LATDbits.LATD5 = 1;
        LATDbits.LATD6 = 1;
        LATDbits.LATD7 = 1;
    } else if (state.main == MAIN_STATE_FAILURE) {
        LATDbits.LATD5 = 1;
        LATDbits.LATD6 = 0;
        LATDbits.LATD7 = 0;
    }
}




/*
 * Does all the ADC stuff.
 * Does measuring all the time, async.
 */
void init_adc(void) {
    ANSELAbits.ANSA0 = 1;
    TRISAbits.TRISA0 = 1;
    adc.entries[ADC_AMP_POSITIVE_VOLTAGE].chs = 0b00000;

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
    ADCON1bits.ADFM = 1;
    ADCON0bits.ADRMD = 0;

    ADCON0bits.ADON = 1;

    adc.current = 0;
    adc.state = 0;

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
            if (ADRESHbits.ADRESH / 16 == 0)
                adc.entries[adc.current].value = (ADRESHbits.ADRESH % 16) * 256 + ADRESLbits.ADRESL + 4096;
            else
                adc.entries[adc.current].value = (ADRESHbits.ADRESH % 16) * 256 + ADRESLbits.ADRESL;

            // Select the next ADC-input to measure
            adc.current++;
            adc.current %= ADC_COUNT;

            // Go back to the beginning to begin measuring again
            adc.state = 0;
        }
    }
}




/*
 * Amplifier control.
 * Only this is allowed to change main.state!
 */
void control_init(void) {
    state.main = MAIN_STATE_STANDBY;
    state.bridged = 0;
    state.power_button = 0;
    state.failure = 0;
}

static long dbg_cancelled = 0;
void control_update(void) {
    if (state.failure) // If any registered failures, go to failure-mode immediately
        state.main = MAIN_STATE_FAILURE;


    if (state.main == MAIN_STATE_STANDBY) {
        if (state.power_button) {
            // Power button is on. We need to boot up
            state.main = MAIN_STATE_POWERON_LOW;
            timer_start(MAIN_STATE_CHANGE_TIMER, 2000);
        }
    } else if (state.main == MAIN_STATE_POWERON_LOW) {
        // Amplifier is warming up with the 160VA/12Vx2-transformer
        if (!state.power_button) { // User cancels power-on
            dbg_cancelled++;
            state.main = MAIN_STATE_POWER_DOWN;
        }

        if (timer_finished(MAIN_STATE_CHANGE_TIMER)) {
            state.main = MAIN_STATE_ON_LOW;
            timer_start(MAIN_STATE_CHANGE_TIMER, 5000);
        }

    } else if (state.main == MAIN_STATE_ON_LOW) {
        if (!state.power_button) { // User cancels power-on
            state.main = MAIN_STATE_POWER_DOWN;
        }

        if (state.power.requiring_high) { // High amount of power needed. We switch transformer
            state.main = MAIN_STATE_POWERON_HIGH;
        }

    } else if (state.main == MAIN_STATE_POWERON_HIGH) {
        if (!state.power_button) { // User cancels power-on
            state.main = MAIN_STATE_POWER_DOWN;
        }

        if (state.power.high_stable) {
            state.main = MAIN_STATE_ON_HIGH;
        }

    } else if (state.main == MAIN_STATE_ON_HIGH) {
        if (!state.power_button) { // User cancels power-on
            state.main = MAIN_STATE_POWER_DOWN;
        }

    } else if (state.main == MAIN_STATE_POWER_DOWN) {
        if (timer_finished(MAIN_STATE_CHANGE_TIMER)) {
            state.main = MAIN_STATE_STANDBY;
            timer_start(MAIN_STATE_CHANGE_TIMER, 500);
        }

    } else if (state.main == MAIN_STATE_FAILURE) {
        // Stuck here forever. User needs to unplug the device to reset state
    }
}




void main(void) {
    system_init();
    time_init();
    //temperature_init();
    buttons_init();
    //init_fans();
    leds_init();
    //init_voltage();
    //init_clip();
    //init_volume();
    //init_speakers();
    //init_adc();
    control_init();

    while (1) {
        time_update();
        //update_adc();
        //temperature_update();
        buttons_update();
        //update_fans();
        leds_update();
        //update_voltage();
        //update_clip();
        //update_volume();
        //update_speakers();
        control_update();

        rounds++;
    }
}
