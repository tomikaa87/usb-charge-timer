/**
  Generated Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This is the main file generated using PIC10 / PIC12 / PIC16 / PIC18 MCUs

  Description:
    This header file provides implementations for driver APIs for all modules selected in the GUI.
    Generation Information :
        Product Revision  :  PIC10 / PIC12 / PIC16 / PIC18 MCUs - 1.81.8
        Device            :  PIC16F1459
        Driver Version    :  2.00
*/

/*
    (c) 2018 Microchip Technology Inc. and its subsidiaries.

    Subject to your compliance with these terms, you may use Microchip software and any
    derivatives exclusively with Microchip products. It is your responsibility to comply with third party
    license terms applicable to your use of third party software (including open source software) that
    may accompany Microchip software.

    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY
    IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS
    FOR A PARTICULAR PURPOSE.

    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP
    HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO
    THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL
    CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT
    OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS
    SOFTWARE.
*/

// CONFIG
#pragma config FOSC = INTOSCIO  // Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA4/OSC2/CLKOUT pin, I/O function on RA5/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = ON       // Power-up Timer Enable bit (PWRT enabled)
#pragma config MCLRE = ON       // MCLR Pin Function Select bit (MCLR pin function is MCLR)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = ON       // Brown Out Detect (BOR enabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)

#define TIMER0_PRELOAD_VALUE            (100)
#define TIMER1_PRELOAD_VALUE            (3036)  // 1 Hz period @ 2 MHz FOSC
#define _XTAL_FREQ                      (2000000)
#define BUTTON_DEBOUNCE_DELAY           (10)
#define LED_PWM_MAX                     (0b111111)
#define LED_COUNT_BLINK_DELAY           (30)
#define LED_COUNT_BLINK_DELAY_LONG      (60)
#define LED_MODE_SWITCH_DELAY           (5)
#define OUTPUT_PIN                      (GP5)

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <xc.h>

static struct Button
{
    volatile uint8_t timer;
    volatile bool pressed;
} button = {
    .timer = 0,
    .pressed = false
};

static struct LED
{
    enum
    {
        LED_MODE_SETUP,
        LED_MODE_OFF,
        LED_MODE_COUNTING,
        LED_MODE_PULSING
    } mode;

    uint8_t timer;
    uint8_t count;
    uint8_t initialCount;
    bool on;
    uint8_t pulseDc;
} led = {
    .mode = LED_MODE_OFF,
    .timer = 0,
    .count = 0,
    .initialCount = 0,
    .on = false,
    .pulseDc = 0
};

static struct Output
{
    uint16_t timer;
    bool on;
} output = {
    .timer = 0,
    .on = false
};

static struct State
{
    enum
    {
        STATE_RESET,
        STATE_OFF,
        STATE_PROGRAM,
        STATE_RUNNING
    } state;
    uint16_t timer;
    uint8_t outputTimerCount;
} state = {
    .state = STATE_RESET,
    .timer = 0,
    .outputTimerCount = 0
};

static bool Button_task(void);

static void LED_task(void);
static void LED_count(uint8_t num);
static void LED_pulse(void);
static void LED_turnOff(void);

static void Output_task(void);
static void Output_turnOff(void);
static void Output_turnOn(uint16_t seconds);
static bool Output_isOn(void);

/*
                         Main application
 */
void main()
{
    // Setup
    OSCCONbits.IRCF = 0b101;    // 2 MHz
    ANSELbits.ANS = 0;          // All pins digital I/O
    OPTION_REGbits.nGPPU = 0;   // Enable WPU
    WPUbits.WPUA2 = 0;          // No WPU on the LED output pin (GP2)
    WPUbits.WPUA5 = 0;          // No WPU on the switch output pin (GP5)
    WPUbits.WPU4 = 1;
    IOCbits.IOC4 = 1;           // IOC on the button input pin (GP4)
    GP2 = 0;
    TRISIObits.TRISIO2 = 0;     // LED output pin direction
    TRISIObits.TRISIO5 = 0;     // Switch output pin direction

    // Timer0: system tick timer, ~10ms period @ 2 MHz
    T0CS = 0;                   // FOSC/4 clock source
    PSA = 0;                    // Prescaler assigned to Timer0
    OPTION_REGbits.PS = 0b100;  // 1:32 prescaler
    TMR0IE = 1;

    // Timer1: main timer
    T1CONbits.T1CKPS = 0b11;    // 1:8 prescale
    TMR1IE = 1;

    // Timer2: PWM timer, 25 kHz @ 2 MHz FOSC
    PR2 = 0b00010011;
    TMR2ON = 1;

    // CCP1: PWM output for the LED, 6-bit resolution
    CCP1CONbits.CCP1M = 0b1100;

    GIE = 1;
    PEIE = 1;
    GPIE = 1;
    IOC4 = 1;
    GPIF = 0;

    Output_turnOff();

    while (1)
    {
        LED_task();
        Output_task();
        bool buttonPressed = Button_task();

        switch (state.state) {
            case STATE_RESET:
                state.outputTimerCount = 0;
                state.state = STATE_OFF;
                LED_turnOff();
                break;

            case STATE_OFF:
                if (buttonPressed) {
                    state.state = STATE_PROGRAM;
                    // Fall through into STATE_PROGRAM
                } else {
                    break;
                }

            case STATE_PROGRAM:
                if (buttonPressed) {
                    if (++state.outputTimerCount > 8) {
                        state.outputTimerCount = 0;
                    }

                    if (state.outputTimerCount == 0) {
                        state.state = STATE_RESET;
                    } else {
                        LED_count(state.outputTimerCount);
                        state.timer = 1000;
                    }
                } else {
                    if (state.timer == 0) {
                        state.state = STATE_RUNNING;
                        Output_turnOn((uint16_t)state.outputTimerCount * 15 * 60);
                        LED_pulse();
                    }
                }
                break;

            case STATE_RUNNING:
                if (buttonPressed) {
                    Output_turnOff();
                }

                if (!Output_isOn()) {
                    state.state = STATE_RESET;
                }
                break;
        }
    }
}

static bool Button_task()
{
    if (button.timer > 0) {
        return false;
    }

    if (button.pressed) {
        button.pressed = false;
        button.timer = BUTTON_DEBOUNCE_DELAY;

        return true;
    }

    return false;
}

static void _LED_setPWMDutyCycle(const uint8_t dc)
{
    CCPR1L = dc >> 2;
    CCP1CONbits.DC1B = dc & 0b11;
}

static void _LED_setup()
{
    _LED_setPWMDutyCycle(0);
}

static void LED_task()
{
    if (led.timer > 0) {
        return;
    }

    switch (led.mode) {
        case LED_MODE_SETUP:
            _LED_setup();
            led.mode = LED_MODE_OFF;
            break;

        case LED_MODE_OFF:
            break;

        case LED_MODE_COUNTING:
            if (led.initialCount == 0) {
                led.mode = LED_MODE_OFF;
                _LED_setPWMDutyCycle(0);
                break;
            }

            if (!led.on) {
                if (led.count == 0) {
                    led.count = led.initialCount;
                }

                --led.count;
                _LED_setPWMDutyCycle(LED_PWM_MAX);
                led.timer = LED_COUNT_BLINK_DELAY;
                led.on = true;
            } else {
                _LED_setPWMDutyCycle(0);
                led.on = false;
                led.timer =
                    led.count == 0
                        ? LED_COUNT_BLINK_DELAY_LONG
                        : LED_COUNT_BLINK_DELAY;
            }

            break;

        case LED_MODE_PULSING:
            _LED_setPWMDutyCycle(led.pulseDc);

            if (led.on) {
                if (++led.pulseDc == LED_PWM_MAX) {
                    led.on = false;
                }
            } else {
                if (--led.pulseDc == 0) {
                    led.on = true;
                }
            }

            led.timer = 2;
            break;
    }
}

static void LED_count(const uint8_t count)
{
    led.mode = LED_MODE_COUNTING;
    led.initialCount = count;
    led.on = false;
    led.timer = LED_MODE_SWITCH_DELAY;
    led.count = 0;
    _LED_setPWMDutyCycle(0);
}

static void LED_pulse()
{
    led.mode = LED_MODE_PULSING;
    led.timer = LED_MODE_SWITCH_DELAY;
    led.on = true;
    led.pulseDc = 0;
    _LED_setPWMDutyCycle(0);
}

static void LED_turnOff()
{
    led.mode = LED_MODE_OFF;
    led.timer = 0;
    _LED_setPWMDutyCycle(0);
}

static void Output_task()
{
    if (output.timer > 0) {
        return;
    }

    if (output.on) {
        Output_turnOff();
    }
}

static void Output_turnOff()
{
    OUTPUT_PIN = 1;
    TRISIObits.TRISIO5 = 1; // Open-drain

    TMR1ON = 0;
    output.on = false;
}

static void Output_turnOn(const uint16_t seconds)
{
    OUTPUT_PIN = 0;
    TRISIObits.TRISIO5 = 0;

    output.timer = seconds;
    output.on = true;

    TMR1H = (uint8_t)(TIMER1_PRELOAD_VALUE >> 8);
    TMR1L = (uint8_t)(TIMER1_PRELOAD_VALUE & 0xff);

    TMR1ON = 1;
}

static bool Output_isOn()
{
    return output.on;
}

void __interrupt() isr()
{
    if (TMR1IE && TMR1IF) {
        TMR1H = (uint8_t)(TIMER1_PRELOAD_VALUE >> 8);
        TMR1L = (uint8_t)(TIMER1_PRELOAD_VALUE & 0xff);
        TMR1IF = 0;

        if (output.timer > 0) {
            --output.timer;
        } else {
            TMR1ON = 0;
            OUTPUT_PIN = 0;
        }
    } else if (TMR0IE && TMR0IF) {
        TMR0 = TIMER0_PRELOAD_VALUE;
        TMR0IF = 0;

        if (button.timer > 0) {
            --button.timer;
        }

        if (led.timer > 0) {
            --led.timer;
        }

        if (state.timer > 0) {
            --state.timer;
        }
    } else if (GPIE && GPIF) {
        if (IOC4 && (GP4 == 0)) {
            button.pressed = true;
        }

        GPIF = 0;
    }
}

/**
 End of File
*/