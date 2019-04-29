/*
 * buttons.c
 *
 *  Created on: Aug 12, 2012, modified 9/8/2017
 *      Author: Gene Bogdanov
 *
 * ECE 3849 Lab button handling
 */
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "driverlib/adc.h"
#include "sysctl_pll.h"
#include "buttons.h"

extern uint32_t gSystemClock;   // [Hz] system clock frequency
extern volatile uint32_t gTime; // time in hundredths of a second
volatile uint32_t buttonfifo[11];
#define FIFO_SIZE 11        // FIFO capacity is 1 item fewer
volatile uint32_t fifo[FIFO_SIZE];  // FIFO storage array
volatile int fifo_head = 0; // index of the first item in the FIFO
volatile int fifo_tail = 0; // index one step past the last item
#define BUTTON_SCAN_RATE 200 // [Hz] button scanning interrupt rate
#define BUTTON_INT_PRIORITY 32 // button interrupt priority

// public globals
volatile uint32_t gButtons = 0; // debounced button state, one per bit in the lowest bits
// button is pressed if its bit is 1, not pressed if 0
uint32_t gJoystick[2] = { 0 };    // joystick coordinates
uint32_t gADCSamplingRate;      // [Hz] actual ADC sampling rate
// imported globals
extern uint32_t gSystemClock;   // [Hz] system clock frequency
extern volatile uint32_t gTime; // time in hundredths of a second
extern volatile uint16_t triggerDirection;
extern volatile uint16_t voltageScaleState;
extern volatile uint16_t samplingRateState;
extern volatile uint16_t mode;
extern volatile float VOLTS_PER_DIV;
// initialize all button and joystick handling hardware
void ButtonInit(void)
{
    // GPIO PJ0 and PJ1 = EK-TM4C1294XL buttons 1 and 2
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);
    GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    GPIOPadConfigSet(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1,
    GPIO_STRENGTH_2MA,
                     GPIO_PIN_TYPE_STD_WPU);

    // analog input AIN13, at GPIO PD2 = BoosterPack Joystick HOR(X)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_2);
    // analog input AIN17, at GPIO PK1 = BoosterPack Joystick VER(Y)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
    GPIOPinTypeADC(GPIO_PORTK_BASE, GPIO_PIN_1);

    //select
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, GPIO_PIN_4);
    GPIOPadConfigSet(GPIO_PORTD_BASE, GPIO_PIN_4,
    GPIO_STRENGTH_2MA,
                     GPIO_PIN_TYPE_STD_WPU);
    //S1
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOH);
    GPIOPinTypeGPIOInput(GPIO_PORTH_BASE, GPIO_PIN_1);
    GPIOPadConfigSet(GPIO_PORTH_BASE, GPIO_PIN_1,
    GPIO_STRENGTH_2MA,
                     GPIO_PIN_TYPE_STD_WPU);
    //S2
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
    GPIOPinTypeGPIOInput(GPIO_PORTK_BASE, GPIO_PIN_6);
    GPIOPadConfigSet(GPIO_PORTK_BASE, GPIO_PIN_6,
    GPIO_STRENGTH_2MA,
                     GPIO_PIN_TYPE_STD_WPU);

    // initialize ADC0 peripheral
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    uint32_t pll_frequency = SysCtlFrequencyGet(CRYSTAL_FREQUENCY);
    uint32_t pll_divisor = (pll_frequency - 1) / (16 * ADC_SAMPLING_RATE) + 1; // round divisor up
    gADCSamplingRate = pll_frequency / (16 * pll_divisor); // actual sampling rate may differ from ADC_SAMPLING_RATE
    ADCClockConfigSet(ADC0_BASE, ADC_CLOCK_SRC_PLL | ADC_CLOCK_RATE_FULL,
                      pll_divisor); // only ADC0 has PLL clock divisor control

    // initialize ADC sampling sequence
    ADCSequenceDisable(ADC0_BASE, 0);
    ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_CH13);  // Joystick HOR(X)
    ADCSequenceStepConfigure(ADC0_BASE, 0, 1,
    ADC_CTL_CH17 | ADC_CTL_IE | ADC_CTL_END); // Joystick VER(Y)
    ADCSequenceEnable(ADC0_BASE, 0);
}

// update the debounced button state gButtons
void ButtonDebounce(uint32_t buttons)
{
    int32_t i, mask;
    static int32_t state[BUTTON_COUNT]; // button state: 0 = released
                                        // BUTTON_PRESSED_STATE = pressed
                                        // in between = previous state
    for (i = 0; i < BUTTON_COUNT; i++)
    {
        mask = 1 << i;
        if (buttons & mask)
        {
            state[i] += BUTTON_STATE_INCREMENT;
            if (state[i] >= BUTTON_PRESSED_STATE)
            {
                state[i] = BUTTON_PRESSED_STATE;
                gButtons |= mask; // update debounced button state
            }
        }
        else
        {
            state[i] -= BUTTON_STATE_DECREMENT;
            if (state[i] <= 0)
            {
                state[i] = 0;
                gButtons &= ~mask;
            }
        }
    }
}

// sample joystick and convert to button presses
void ButtonReadJoystick(void)
{
    ADCProcessorTrigger(ADC0_BASE, 0); // trigger the ADC sample sequence for Joystick X and Y
    while (!ADCIntStatus(ADC0_BASE, 0, false))
        ;  // wait until the sample sequence has completed
    ADCSequenceDataGet(ADC0_BASE, 0, gJoystick);  // retrieve joystick data
    ADCIntClear(ADC0_BASE, 0);              // clear ADC sequence interrupt flag

    // process joystick movements as button presses using hysteresis
    if (gJoystick[0] > JOYSTICK_UPPER_PRESS_THRESHOLD)
        gButtons |= 1 << 5; // joystick right in position 5
    if (gJoystick[0] < JOYSTICK_UPPER_RELEASE_THRESHOLD)
        gButtons &= ~(1 << 5);

    if (gJoystick[0] < JOYSTICK_LOWER_PRESS_THRESHOLD)
        gButtons |= 1 << 6; // joystick left in position 6
    if (gJoystick[0] > JOYSTICK_LOWER_RELEASE_THRESHOLD)
        gButtons &= ~(1 << 6);

    if (gJoystick[1] > JOYSTICK_UPPER_PRESS_THRESHOLD)
        gButtons |= 1 << 7; // joystick up in position 7
    if (gJoystick[1] < JOYSTICK_UPPER_RELEASE_THRESHOLD)
        gButtons &= ~(1 << 7);

    if (gJoystick[1] < JOYSTICK_LOWER_PRESS_THRESHOLD)
        gButtons |= 1 << 8; // joystick down in position 8
    if (gJoystick[1] > JOYSTICK_LOWER_RELEASE_THRESHOLD)
        gButtons &= ~(1 << 8);

    if (~GPIOPinRead(GPIO_PORTH_BASE, 0xff) & (GPIO_PIN_1))
        gButtons |= 1 << 2;
    else
//    if(~(~GPIOPinRead(GPIO_PORTH_BASE, 0xff) & (GPIO_PIN_1)))
        gButtons &= ~(1 << 2);

    if (~GPIOPinRead(GPIO_PORTK_BASE, 0xff) & (GPIO_PIN_6))
        gButtons |= 1 << 3;
    else
//    if(~(~GPIOPinRead(GPIO_PORTK_BASE, 0xff) & (GPIO_PIN_6)))
        gButtons &= ~(1 << 3);

    if (~GPIOPinRead(GPIO_PORTD_BASE, 0xff) & (GPIO_PIN_4))
        gButtons |= 1 << 4;
    else
//    if(~(~GPIOPinRead(GPIO_PORTD_BASE, 0xff) & (GPIO_PIN_4)))
        gButtons &= ~(1 << 4);
}

// autorepeat button presses if a button is held long enough
uint32_t ButtonAutoRepeat(void)
{
    static int count[BUTTON_AND_JOYSTICK_COUNT] = { 0 }; // autorepeat counts
    int i;
    uint32_t mask;
    uint32_t presses = 0;
    for (i = 0; i < BUTTON_AND_JOYSTICK_COUNT; i++)
    {
        mask = 1 << i;
        if (gButtons & mask)
            count[i]++;     // increment count if button is held
        else
            count[i] = 0;   // reset count if button is let go
        if (count[i] >= BUTTON_AUTOREPEAT_INITIAL
                && (count[i] - BUTTON_AUTOREPEAT_INITIAL)
                        % BUTTON_AUTOREPEAT_NEXT == 0)
            presses |= mask;    // register a button press due to auto-repeat
    }
    return presses;
}

// ISR for scanning and debouncing buttons
uint32_t getButton(void)
{
//    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT); // clear interrupt flag
    // read hardware button state
    uint32_t gpio_buttons = ~GPIOPinRead(GPIO_PORTJ_BASE, 0xff)
            & (GPIO_PIN_0 | GPIO_PIN_1); // EK-TM4C1294XL buttons in positions 0 and 1
    uint32_t old_buttons = gButtons;    // save previous button state
    ButtonDebounce(gpio_buttons); // Run the button debouncer. The result is in gButtons.
    ButtonReadJoystick(); // Convert joystick state to button presses. The result is in gButtons.
    uint32_t presses = ~old_buttons & gButtons; // detect button presses (transitions from not pressed to pressed)
    presses |= ButtonAutoRepeat(); // autorepeat presses if a button is held long enough
    return presses;
//
//    static bool tic = false;
//    static bool running = true;
//
//    ButtonPress(presses);
//    if (running)
//    {
//        if (tic)
//            gTime++; // increment time every other ISR call
//        tic = !tic;
//    }
}

int fifo_put(uint32_t data)
{
    int new_tail = fifo_tail + 1;
    if (new_tail >= FIFO_SIZE)
    {
        new_tail = 0; // Wrap around
    }
    if (fifo_head != new_tail)
    {    // If the FIFO is not full
        fifo[fifo_tail] = data;    // Store data into the FIFO
        fifo_tail = new_tail;    // Advance FIFO tail index
        return 1;    // Success
    }
    return 0;   // Full
}

// get data from the FIFO
// returns 1 on success, 0 if FIFO was empty
int fifo_get(uint32_t *data)
{
    if (fifo_head != fifo_tail)  // If the FIFO is not empty
    {
        *data = fifo[fifo_head];    // Read data from the FIFO
        IntMasterDisable();
        fifo_head++;    // Advance FIFO head index
        if (fifo_head >= FIFO_SIZE)
            fifo_head = 0;    // Wrap around
        IntMasterEnable();
        return 1;    // Success
    }
    return 0;   // empty
}

// Switch voltage scale with button press
void changeVoltageRate(int state)
{
        switch (state)
        {
        case 0:
            VOLTS_PER_DIV = .1;
            break;
        case 1:
            VOLTS_PER_DIV = .2;
            break;
        case 2:
            VOLTS_PER_DIV = .5;
            break;
        case 3:
            VOLTS_PER_DIV = 1;
            break;
        }
}

void ButtonPress(uint32_t presses)
{
    if (presses & 1) // EK-TM4C1294XL button 1 pressed, toggle trigger direction
    {
        triggerDirection = !triggerDirection;
    }
    if (presses & 2) // EK-TM4C1294XL button 2 pressed, toggle between voltage scale
    {
        if (voltageScaleState < 3)
        {
            voltageScaleState++;
        }
        else
        {
            voltageScaleState = 1;
        }
        changeVoltageRate(voltageScaleState);
    }
    if (presses & 4) // Boosterpack MK-II button 1 pressed, increase time scale
    {
        if (samplingRateState > 0)
        {
            samplingRateState--;
        }
        else
        {
            samplingRateState = 11;
        }
        changeSamplingRate(samplingRateState);
    }
    if (presses & 8) // Boosterpack MK-II button 2 pressed, decrease time scale
    {
        if (samplingRateState < 11)
        {
            samplingRateState++;
        }
        else
        {
            samplingRateState = 0;
        }
        changeSamplingRate(samplingRateState);
    }
    if (presses & 16) // Select button press, change mode
    {
        mode = !mode;
    }
}
// Change time scale by changing the sampling rate of the ADC through changing the load of the timer.
void changeSamplingRate(int state)
{
    ADCSequenceDisable(ADC1_BASE, 0); // Disable ADC before changing time scale
    TimerDisable(TIMER2_BASE, TIMER_A); // Disable timer A2
    ADCSequenceConfigure(ADC1_BASE, 0, ADC_TRIGGER_TIMER, 1); // Use Timer A2 to trigger ADC conversions
    switch (state)
    {
    case 0: // 100ms
        TimerLoadSet(TIMER2_BASE, TIMER_A, 500000); // Set timer load to 500000
        TimerControlTrigger(TIMER2_BASE, TIMER_A, 1);
        break;

    case 1: // 50ms
        TimerLoadSet(TIMER2_BASE, TIMER_A, 250000);
        TimerControlTrigger(TIMER2_BASE, TIMER_A, 1);
        break;

    case 2: // 20ms
        TimerLoadSet(TIMER2_BASE, TIMER_A, 100000);
        TimerControlTrigger(TIMER2_BASE, TIMER_A, 1);
        break;

    case 3: // 10ms
        TimerLoadSet(TIMER2_BASE, TIMER_A, 50000);
        TimerControlTrigger(TIMER2_BASE, TIMER_A, 1);
        break;

    case 4: // 5ms
        TimerLoadSet(TIMER2_BASE, TIMER_A, 25000);
        TimerControlTrigger(TIMER2_BASE, TIMER_A, 1);
        break;

    case 5: // 2ms
        TimerLoadSet(TIMER2_BASE, TIMER_A, 10000);
        TimerControlTrigger(TIMER2_BASE, TIMER_A, 1);
        break;

    case 6: // 1ms
        TimerLoadSet(TIMER2_BASE, TIMER_A, 5000);
        TimerControlTrigger(TIMER2_BASE, TIMER_A, 1);
        break;

    case 7: // 500us
        TimerLoadSet(TIMER2_BASE, TIMER_A, 2500);
        TimerControlTrigger(TIMER2_BASE, TIMER_A, 1);
        break;

    case 8: // 200us
        TimerLoadSet(TIMER2_BASE, TIMER_A, 1000);
        TimerControlTrigger(TIMER2_BASE, TIMER_A, 1);
        break;

    case 9: // 100us
        TimerLoadSet(TIMER2_BASE, TIMER_A, 500);
        TimerControlTrigger(TIMER2_BASE, TIMER_A, 1);
        break;

    case 10: // 50us
        TimerLoadSet(TIMER2_BASE, TIMER_A, 250);
        TimerControlTrigger(TIMER2_BASE, TIMER_A, 1);
        break;

    case 11: // 20us
        ADCSequenceConfigure(ADC1_BASE, 0, ADC_TRIGGER_ALWAYS, 0); // 20us time scale, default; specify the "Always" trigger
        TimerControlTrigger(TIMER2_BASE, TIMER_A, 0);
        break;
    }
    ADCSequenceEnable(ADC1_BASE, 0); // Enable the ADC
    TimerEnable(TIMER2_BASE, TIMER_BOTH); // Enable Timer A2
}
